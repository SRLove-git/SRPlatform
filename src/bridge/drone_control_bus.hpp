#pragma once

#include "bridge/actuator_bus.hpp"
#include "bridge/drone_entity.hpp"
#include "bridge/sensor_bus.hpp"

namespace srp::bridge
{

constexpr MotorId kThrottleMotorId = 1;

constexpr SensorId kAltitudeSensorId = 1;
constexpr SensorId kVerticalVelocitySensorId = 2;

// Adapts the drone collective throttle to the generic actuator bus.
class DroneActuatorBus final : public IActuatorBus
{
public:
    explicit DroneActuatorBus(DroneEntity& drone);

    void setMotor(MotorId id, ActuatorValue value) override;
    void setServo(ServoId id, ActuatorValue angle) override;

private:
    DroneEntity* drone_;
};

// Exposes drone state through the generic sensor bus.
class DroneSensorBus final : public ISensorBus
{
public:
    explicit DroneSensorBus(const DroneEntity& drone);

    SensorValue read(SensorId id) const override;

private:
    const DroneEntity* drone_;
};

}  // namespace srp::bridge
