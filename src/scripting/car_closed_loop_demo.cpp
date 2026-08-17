#include "scripting/car_closed_loop_demo.hpp"

#include "bridge/bridge.hpp"
#include "bridge/car_control_bus.hpp"
#include "scripting/lua_script_host.hpp"

#include <memory>

namespace srp::scripting
{

struct CarClosedLoopDemo::Impl
{
    bridge::CarEntity car;
    std::shared_ptr<bridge::CarActuatorBus> actuator_bus;
    std::shared_ptr<bridge::CarSensorBus> sensor_bus;
    std::shared_ptr<bridge::Bridge> bridge;
    LuaScriptHost host;

    explicit Impl(const bridge::CarParameters& parameters)
        : car(parameters),
          actuator_bus(std::make_shared<bridge::CarActuatorBus>(car)),
          sensor_bus(std::make_shared<bridge::CarSensorBus>(car)),
          bridge(std::make_shared<bridge::Bridge>())
    {
        bridge->attachActuatorBus(actuator_bus);
        bridge->attachSensorBus(sensor_bus);
        host.bindControl(bridge);
    }
};

CarClosedLoopDemo::CarClosedLoopDemo(
    const bridge::CarParameters& parameters)
    : impl_(std::make_unique<Impl>(parameters))
{
}

CarClosedLoopDemo::~CarClosedLoopDemo() = default;

bridge::CarEntity& CarClosedLoopDemo::car()
{
    return impl_->car;
}

const bridge::CarEntity& CarClosedLoopDemo::car() const
{
    return impl_->car;
}

LuaScriptHost& CarClosedLoopDemo::host()
{
    return impl_->host;
}

bool CarClosedLoopDemo::loadScript(
    const std::string& id,
    const std::string& source)
{
    return impl_->host.load(id, source);
}

bool CarClosedLoopDemo::step(double dt)
{
    const bool script_ok = impl_->host.runOnce(dt);
    impl_->car.step(dt);
    return script_ok;
}

}  // namespace srp::scripting
