#include "bridge/car_control_bus.hpp"

#include "physics/rigid_body.hpp"

namespace srp::bridge
{

CarActuatorBus::CarActuatorBus(CarEntity& car)
    : car_(&car)
{
}

void CarActuatorBus::setMotor(MotorId id, ActuatorValue value)
{
    if (id == kDriveMotorId && car_ != nullptr)
    {
        car_->setThrottle(value);
    }
}

void CarActuatorBus::setServo(ServoId id, ActuatorValue angle)
{
    if (id == kSteeringServoId && car_ != nullptr)
    {
        car_->setSteering(angle);
    }
}

CarSensorBus::CarSensorBus(const CarEntity& car)
    : car_(&car)
{
}

SensorValue CarSensorBus::read(SensorId id) const
{
    if (car_ == nullptr)
    {
        return 0.0;
    }

    const physics::RigidBodyState* chassis = car_->chassisBody();

    switch (id)
    {
    case kSpeedSensorId:
        return chassis != nullptr ? chassis->linear_velocity.x : 0.0;
    case kPositionSensorId:
        return chassis != nullptr ? chassis->position.x : 0.0;
    case kHeadingSensorId:
        return car_->heading();
    default:
        return 0.0;
    }
}

}  // namespace srp::bridge
