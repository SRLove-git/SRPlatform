#pragma once

#include "bridge/bridge_types.hpp"
#include "scripting/script_host.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace srp::bridge
{
class Bridge;
}

namespace srp::scripting
{

class LuaScriptHost final : public IScriptHost
{
public:
    LuaScriptHost();
    ~LuaScriptHost() override;

    LuaScriptHost(const LuaScriptHost&) = delete;
    LuaScriptHost& operator=(const LuaScriptHost&) = delete;
    LuaScriptHost(LuaScriptHost&&) = delete;
    LuaScriptHost& operator=(LuaScriptHost&&) = delete;

    bool load(const std::string& id, const std::string& source) override;
    bool reload(const std::string& id) override;
    bool runOnce(double dt) override;

    bool hasScript(const std::string& id) const override;
    std::size_t scriptCount() const override;

    void bindControl(std::shared_ptr<bridge::Bridge> bridge);

    // Registers an external sensor provider consulted before the bridge when
    // read_sensor() is called. Used by courses to expose sensors that live in
    // the editor layer (e.g. distance sensors aimed at the editable scene).
    void setSensorOverride(
        bridge::SensorId id,
        std::function<std::optional<double>()> provider);
    void clearSensorOverrides();

    std::optional<std::string> lastError() const;
    std::optional<double> getNumber(
        const std::string& id,
        const std::string& name) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::scripting
