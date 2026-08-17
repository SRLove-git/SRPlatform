#include "bridge/arm_control_bus.hpp"

namespace srp::bridge
{

ArmActuatorBus::ArmActuatorBus(ArmEntity& arm)
    : arm_(&arm)
{
}

void ArmActuatorBus::setMotor(MotorId, ActuatorValue)
{
    // The arm has no motors; servo commands drive the joints.
}

void ArmActuatorBus::setServo(ServoId id, ActuatorValue angle)
{
    if (arm_ == nullptr || id == kInvalidServoId)
    {
        return;
    }
    arm_->setServo(static_cast<std::size_t>(id - 1), angle);
}

ArmSensorBus::ArmSensorBus(const ArmEntity& arm)
    : arm_(&arm)
{
}

SensorValue ArmSensorBus::read(SensorId id) const
{
    if (arm_ == nullptr || id == kInvalidSensorId)
    {
        return 0.0;
    }
    return arm_->jointAngle(static_cast<std::size_t>(id - 1));
}

}  // namespace srp::bridge
