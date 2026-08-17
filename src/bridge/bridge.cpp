#include "bridge/bridge.hpp"

#include <utility>

namespace srp::bridge
{

void Bridge::attachActuatorBus(std::shared_ptr<IActuatorBus> bus)
{
    actuator_bus_ = std::move(bus);
}

void Bridge::attachSensorBus(std::shared_ptr<ISensorBus> bus)
{
    sensor_bus_ = std::move(bus);
}

void Bridge::detachActuatorBus()
{
    actuator_bus_.reset();
}

void Bridge::detachSensorBus()
{
    sensor_bus_.reset();
}

bool Bridge::hasActuatorBus() const
{
    return static_cast<bool>(actuator_bus_);
}

bool Bridge::hasSensorBus() const
{
    return static_cast<bool>(sensor_bus_);
}

bool Bridge::setMotor(MotorId id, ActuatorValue value)
{
    if (!actuator_bus_)
    {
        return false;
    }

    actuator_bus_->setMotor(id, value);
    return true;
}

bool Bridge::setServo(ServoId id, ActuatorValue angle)
{
    if (!actuator_bus_)
    {
        return false;
    }

    actuator_bus_->setServo(id, angle);
    return true;
}

std::optional<SensorValue> Bridge::readSensor(SensorId id) const
{
    if (!sensor_bus_)
    {
        return std::nullopt;
    }

    return sensor_bus_->read(id);
}

}  // namespace srp::bridge
