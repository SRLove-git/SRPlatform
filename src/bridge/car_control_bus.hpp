#pragma once

#include "bridge/actuator_bus.hpp"
#include "bridge/car_entity.hpp"
#include "bridge/sensor_bus.hpp"

namespace srp::bridge
{

constexpr MotorId kDriveMotorId = 1;
constexpr ServoId kSteeringServoId = 1;

constexpr SensorId kSpeedSensorId = 1;
constexpr SensorId kPositionSensorId = 2;
constexpr SensorId kHeadingSensorId = 3;

// Adapts the car throttle and steering controls to the generic actuator bus
// used by the scripting layer.
class CarActuatorBus final : public IActuatorBus
{
public:
    explicit CarActuatorBus(CarEntity& car);

    void setMotor(MotorId id, ActuatorValue value) override;
    void setServo(ServoId id, ActuatorValue angle) override;

private:
    CarEntity* car_;
};

// Exposes observable car state through the generic sensor bus.
class CarSensorBus final : public ISensorBus
{
public:
    explicit CarSensorBus(const CarEntity& car);

    SensorValue read(SensorId id) const override;

private:
    const CarEntity* car_;
};

}  // namespace srp::bridge
