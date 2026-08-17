#include "scripting/drone_closed_loop_demo.hpp"

#include "bridge/bridge.hpp"
#include "bridge/drone_control_bus.hpp"
#include "scripting/lua_script_host.hpp"

#include <memory>

namespace srp::scripting
{

struct DroneClosedLoopDemo::Impl
{
    bridge::DroneEntity drone;
    std::shared_ptr<bridge::DroneActuatorBus> actuator_bus;
    std::shared_ptr<bridge::DroneSensorBus> sensor_bus;
    std::shared_ptr<bridge::Bridge> bridge;
    LuaScriptHost host;

    explicit Impl(const bridge::DroneParameters& parameters)
        : drone(parameters),
          actuator_bus(std::make_shared<bridge::DroneActuatorBus>(drone)),
          sensor_bus(std::make_shared<bridge::DroneSensorBus>(drone)),
          bridge(std::make_shared<bridge::Bridge>())
    {
        bridge->attachActuatorBus(actuator_bus);
        bridge->attachSensorBus(sensor_bus);
        host.bindControl(bridge);
    }
};

DroneClosedLoopDemo::DroneClosedLoopDemo(
    const bridge::DroneParameters& parameters)
    : impl_(std::make_unique<Impl>(parameters))
{
}

DroneClosedLoopDemo::~DroneClosedLoopDemo() = default;

bridge::DroneEntity& DroneClosedLoopDemo::drone()
{
    return impl_->drone;
}

const bridge::DroneEntity& DroneClosedLoopDemo::drone() const
{
    return impl_->drone;
}

LuaScriptHost& DroneClosedLoopDemo::host()
{
    return impl_->host;
}

bool DroneClosedLoopDemo::loadScript(
    const std::string& id,
    const std::string& source)
{
    return impl_->host.load(id, source);
}

bool DroneClosedLoopDemo::step(double dt)
{
    const bool script_ok = impl_->host.runOnce(dt);
    impl_->drone.step(dt);
    return script_ok;
}

}  // namespace srp::scripting
