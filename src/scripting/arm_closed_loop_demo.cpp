#include "scripting/arm_closed_loop_demo.hpp"

#include "bridge/arm_control_bus.hpp"
#include "bridge/bridge.hpp"
#include "scripting/lua_script_host.hpp"

#include <memory>

namespace srp::scripting
{

struct ArmClosedLoopDemo::Impl
{
    bridge::ArmEntity arm;
    std::shared_ptr<bridge::ArmActuatorBus> actuator_bus;
    std::shared_ptr<bridge::ArmSensorBus> sensor_bus;
    std::shared_ptr<bridge::Bridge> bridge;
    LuaScriptHost host;

    explicit Impl(const bridge::ArmParameters& parameters)
        : arm(parameters),
          actuator_bus(std::make_shared<bridge::ArmActuatorBus>(arm)),
          sensor_bus(std::make_shared<bridge::ArmSensorBus>(arm)),
          bridge(std::make_shared<bridge::Bridge>())
    {
        bridge->attachActuatorBus(actuator_bus);
        bridge->attachSensorBus(sensor_bus);
        host.bindControl(bridge);
    }
};

ArmClosedLoopDemo::ArmClosedLoopDemo(
    const bridge::ArmParameters& parameters)
    : impl_(std::make_unique<Impl>(parameters))
{
}

ArmClosedLoopDemo::~ArmClosedLoopDemo() = default;

bridge::ArmEntity& ArmClosedLoopDemo::arm()
{
    return impl_->arm;
}

const bridge::ArmEntity& ArmClosedLoopDemo::arm() const
{
    return impl_->arm;
}

LuaScriptHost& ArmClosedLoopDemo::host()
{
    return impl_->host;
}

bool ArmClosedLoopDemo::loadScript(
    const std::string& id,
    const std::string& source)
{
    return impl_->host.load(id, source);
}

bool ArmClosedLoopDemo::step(double dt)
{
    const bool script_ok = impl_->host.runOnce(dt);
    impl_->arm.step(dt);
    return script_ok;
}

}  // namespace srp::scripting
