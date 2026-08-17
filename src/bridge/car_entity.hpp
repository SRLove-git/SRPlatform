#pragma once

#include "bridge/battery_model.hpp"
#include "bridge/dc_motor_model.hpp"
#include "bridge/state_recorder.hpp"

#include <memory>

namespace srp::physics
{
class PhysicsWorld;
struct RigidBodyState;
}

namespace srp::bridge
{

struct CarParameters
{
    BatteryParameters battery;
    DcMotorParameters motor;
    double wheel_radius{0.3};
    double chassis_mass{1.0};
    double wheel_mass{1.0};
    double rolling_resistance_torque_nm{0.01};
    double viscous_load_nm_per_rad_s{0.0005};
};

// A small example vehicle that composes a battery, a DC motor, a chassis, a
// wheel, and the wheel-ground constraint into one stepping unit.
class CarEntity
{
public:
    explicit CarEntity(const CarParameters& parameters = {});
    ~CarEntity();

    CarEntity(const CarEntity&) = delete;
    CarEntity& operator=(const CarEntity&) = delete;
    CarEntity(CarEntity&&) = delete;
    CarEntity& operator=(CarEntity&&) = delete;

    void setThrottle(double value);
    double throttle() const;

    void step(double dt);

    double batteryStateOfCharge() const;
    double batteryVoltage() const;
    double motorCurrent() const;
    double motorAngularVelocity() const;
    double elapsedTime() const;

    const physics::RigidBodyState* chassisBody() const;
    const physics::RigidBodyState* wheelBody() const;
    const StateRecorder& recorder() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::bridge
