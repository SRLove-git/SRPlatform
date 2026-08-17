#include "scripting/lua_script_host.hpp"

#include "bridge/bridge.hpp"

#include <sol/sol.hpp>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace srp::scripting
{

namespace
{

struct Script
{
    std::string source;
    sol::environment environment;
    std::optional<sol::protected_function> update;
};

}  // namespace

struct LuaScriptHost::Impl
{
    sol::state lua;
    std::unordered_map<std::string, std::unique_ptr<Script>> scripts;
    std::shared_ptr<bridge::Bridge> bridge;
    std::string last_error;

    Impl()
    {
        lua.open_libraries(
            sol::lib::base,
            sol::lib::math,
            sol::lib::string,
            sol::lib::table);
    }

    std::unique_ptr<Script> compile(const std::string& source, std::string& error)
    {
        auto script = std::make_unique<Script>();
        script->source = source;

        sol::environment environment(lua, sol::create, lua.globals());

        environment.set_function(
            "read_sensor",
            [this](bridge::SensorId id) -> bridge::SensorValue
            {
                if (bridge == nullptr || !bridge->hasSensorBus())
                {
                    return std::numeric_limits<double>::quiet_NaN();
                }

                return bridge->readSensor(id).value_or(
                    std::numeric_limits<double>::quiet_NaN());
            });

        environment.set_function(
            "set_motor",
            [this](bridge::MotorId id, bridge::ActuatorValue value)
            {
                return bridge != nullptr && bridge->setMotor(id, value);
            });

        environment.set_function(
            "set_servo",
            [this](bridge::ServoId id, bridge::ActuatorValue angle)
            {
                return bridge != nullptr && bridge->setServo(id, angle);
            });

        const sol::protected_function_result result =
            lua.safe_script(source, environment, sol::script_pass_on_error);
        if (!result.valid())
        {
            const sol::error lua_error = result;
            error = lua_error.what();
            return nullptr;
        }

        const sol::object update_object = environment["update"];
        if (update_object.is<sol::protected_function>())
        {
            script->update = update_object.as<sol::protected_function>();
        }

        script->environment = std::move(environment);
        return script;
    }
};

LuaScriptHost::LuaScriptHost()
    : impl_(std::make_unique<Impl>())
{
}

LuaScriptHost::~LuaScriptHost() = default;

bool LuaScriptHost::load(const std::string& id, const std::string& source)
{
    if (id.empty())
    {
        impl_->last_error = "script id cannot be empty";
        return false;
    }

    std::string error;
    std::unique_ptr<Script> script = impl_->compile(source, error);
    if (script == nullptr)
    {
        impl_->last_error = std::move(error);
        return false;
    }

    impl_->scripts[id] = std::move(script);
    impl_->last_error.clear();
    return true;
}

bool LuaScriptHost::reload(const std::string& id)
{
    const auto script_it = impl_->scripts.find(id);
    if (script_it == impl_->scripts.end())
    {
        impl_->last_error = "script not found";
        return false;
    }

    const std::string source = script_it->second->source;
    std::string error;
    std::unique_ptr<Script> script = impl_->compile(source, error);
    if (script == nullptr)
    {
        impl_->last_error = std::move(error);
        return false;
    }

    script_it->second = std::move(script);
    impl_->last_error.clear();
    return true;
}

bool LuaScriptHost::runOnce(double dt)
{
    for (const auto& [id, script] : impl_->scripts)
    {
        if (!script->update.has_value())
        {
            continue;
        }

        const sol::protected_function_result result = (*script->update)(dt);
        if (!result.valid())
        {
            const sol::error lua_error = result;
            impl_->last_error = lua_error.what();
            return false;
        }
    }

    impl_->last_error.clear();
    return true;
}

bool LuaScriptHost::hasScript(const std::string& id) const
{
    return impl_->scripts.find(id) != impl_->scripts.end();
}

std::size_t LuaScriptHost::scriptCount() const
{
    return impl_->scripts.size();
}

void LuaScriptHost::bindControl(std::shared_ptr<bridge::Bridge> bridge)
{
    impl_->bridge = std::move(bridge);
}

std::optional<std::string> LuaScriptHost::lastError() const
{
    if (impl_->last_error.empty())
    {
        return std::nullopt;
    }

    return impl_->last_error;
}

std::optional<double> LuaScriptHost::getNumber(
    const std::string& id,
    const std::string& name) const
{
    const auto script_it = impl_->scripts.find(id);
    if (script_it == impl_->scripts.end())
    {
        return std::nullopt;
    }

    const sol::object value = script_it->second->environment[name];
    if (value.get_type() != sol::type::number)
    {
        return std::nullopt;
    }

    return value.as<double>();
}

}  // namespace srp::scripting
