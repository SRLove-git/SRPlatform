#include "scripting/mod_entity_demo.hpp"

#include "bridge/bridge.hpp"
#include "bridge/car_control_bus.hpp"
#include "bridge/car_entity.hpp"
#include "bridge/drone_control_bus.hpp"
#include "bridge/drone_entity.hpp"
#include "bridge/entity_factory.hpp"
#include "mod/entity_blueprint.hpp"
#include "mod/mod_package.hpp"

#include <fstream>
#include <utility>

namespace srp::scripting
{

struct ModEntityDemo::Impl
{
    mod::ModPackage package;
    std::unique_ptr<bridge::IEntity> entity;
    std::shared_ptr<bridge::Bridge> bridge;
    std::shared_ptr<bridge::IActuatorBus> actuator_bus;
    std::shared_ptr<bridge::ISensorBus> sensor_bus;
    HotReloadScriptHost scripts;
};

ModEntityDemo::ModEntityDemo()
    : impl_(std::make_unique<Impl>())
{
}

ModEntityDemo::~ModEntityDemo() = default;

bool ModEntityDemo::load(
    const std::filesystem::path& mod_directory,
    std::string& error)
{
    error.clear();

    const std::optional<mod::ModPackage> package =
        mod::loadModPackage(mod_directory, error);
    if (!package.has_value())
    {
        return false;
    }
    impl_->package = *package;

    std::filesystem::path blueprint_path =
        impl_->package.root / impl_->package.manifest.blueprint;
    if (impl_->package.manifest.blueprint.empty())
    {
        blueprint_path = impl_->package.root / "blueprint.json";
    }

    std::ifstream blueprint_stream(blueprint_path);
    if (!blueprint_stream.is_open())
    {
        error = "cannot open entity blueprint: " + blueprint_path.string();
        return false;
    }

    nlohmann::json blueprint_json;
    try
    {
        blueprint_stream >> blueprint_json;
    }
    catch (const nlohmann::json::parse_error& parse_error)
    {
        error = std::string("invalid JSON in entity blueprint: ") +
                parse_error.what();
        return false;
    }

    const std::optional<mod::EntityBlueprint> blueprint =
        mod::parseEntityBlueprint(blueprint_json, error);
    if (!blueprint.has_value())
    {
        return false;
    }

    impl_->entity = bridge::createEntity(*blueprint, error);
    if (impl_->entity == nullptr)
    {
        return false;
    }

    impl_->bridge = std::make_shared<bridge::Bridge>();
    if (auto* car = dynamic_cast<bridge::CarEntity*>(impl_->entity.get()))
    {
        impl_->actuator_bus =
            std::make_shared<bridge::CarActuatorBus>(*car);
        impl_->sensor_bus =
            std::make_shared<bridge::CarSensorBus>(*car);
    }
    else if (auto* drone = dynamic_cast<bridge::DroneEntity*>(impl_->entity.get()))
    {
        impl_->actuator_bus =
            std::make_shared<bridge::DroneActuatorBus>(*drone);
        impl_->sensor_bus =
            std::make_shared<bridge::DroneSensorBus>(*drone);
    }
    else
    {
        error = "entity kind has no script buses: " +
                std::string(impl_->entity->kind());
        return false;
    }

    impl_->bridge->attachActuatorBus(impl_->actuator_bus);
    impl_->bridge->attachSensorBus(impl_->sensor_bus);
    impl_->scripts.bindControl(impl_->bridge);

    if (!impl_->scripts.loadFromFile(
            "entry",
            impl_->package.entryPath()))
    {
        error = impl_->scripts.lastError().value_or(
            "failed to load mod entry script");
        return false;
    }

    return true;
}

bool ModEntityDemo::step(double dt)
{
    if (impl_->entity == nullptr)
    {
        return false;
    }

    const bool script_ok = impl_->scripts.runOnce(dt);
    impl_->entity->step(dt);
    return script_ok;
}

bool ModEntityDemo::pollReloads()
{
    return impl_->scripts.pollReloads();
}

bridge::IEntity* ModEntityDemo::entity()
{
    return impl_->entity.get();
}

const bridge::IEntity* ModEntityDemo::entity() const
{
    return impl_->entity.get();
}

HotReloadScriptHost& ModEntityDemo::scripts()
{
    return impl_->scripts;
}

const HotReloadScriptHost& ModEntityDemo::scripts() const
{
    return impl_->scripts;
}

}  // namespace srp::scripting
