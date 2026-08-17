#pragma once

#include "bridge/actuator_bus.hpp"
#include "bridge/arm_entity.hpp"
#include "bridge/sensor_bus.hpp"

namespace srp::bridge
{

constexpr ServoId kArmJoint1ServoId = 1;
constexpr SensorId kArmJoint1SensorId = 1;

// Maps servo ids to arm joint targets (value -1..1 maps to the joint limit).
class ArmActuatorBus final : public IActuatorBus
{
public:
    explicit ArmActuatorBus(ArmEntity& arm);

    void setMotor(MotorId id, ActuatorValue value) override;
    void setServo(ServoId id, ActuatorValue angle) override;

private:
    ArmEntity* arm_;
};

// Exposes joint angles in radians through the generic sensor bus.
class ArmSensorBus final : public ISensorBus
{
public:
    explicit ArmSensorBus(const ArmEntity& arm);

    SensorValue read(SensorId id) const override;

private:
    const ArmEntity* arm_;
};

}  // namespace srp::bridge
