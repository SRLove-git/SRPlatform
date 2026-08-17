#include "bridge/drone_control_bus.hpp"

namespace srp::bridge
{

DroneActuatorBus::DroneActuatorBus(DroneEntity& drone)
    : drone_(&drone)
{
}

void DroneActuatorBus::setMotor(MotorId id, ActuatorValue value)
{
    if (id == kThrottleMotorId && drone_ != nullptr)
    {
        drone_->setThrottle(value);
    }
}

void DroneActuatorBus::setServo(ServoId id, ActuatorValue angle)
{
    (void)id;
    (void)angle;
}

DroneSensorBus::DroneSensorBus(const DroneEntity& drone)
    : drone_(&drone)
{
}

SensorValue DroneSensorBus::read(SensorId id) const
{
    if (drone_ == nullptr)
    {
        return 0.0;
    }

    switch (id)
    {
    case kAltitudeSensorId:
        return drone_->altitude();
    case kVerticalVelocitySensorId:
        return drone_->verticalVelocity();
    default:
        return 0.0;
    }
}

}  // namespace srp::bridge
