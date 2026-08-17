#pragma once

#include "bridge/distance_sensor_model.hpp"
#include "bridge/imu_model.hpp"
#include "bridge/quadcopter_model.hpp"
#include "physics/collision_shape.hpp"

#include <memory>

namespace srp::physics
{
struct RigidBodyState;
}

namespace srp::bridge
{

struct DroneParameters
{
    QuadcopterParameters quadcopter{};
    double chassis_mass{0.6};
    double max_rotor_angular_velocity_rad_s{1600.0};

    DroneParameters()
    {
        // A 6-inch prop gives a comfortable hover throttle margin for the
        // default 0.6 kg craft.
        quadcopter.propeller.diameter_m = 0.15;
    }
};

// A small example quadcopter that composes a rigid body, four propellers,
// an IMU, and a downward distance sensor into one stepping unit.
//
// setThrottle() drives all four rotors together; the body is kept level so
// the lift points straight up, which makes the altitude-hold demo focus on
// the vertical loop. Attitude stabilization is intentionally left to later
// phases.
class DroneEntity
{
public:
    explicit DroneEntity(const DroneParameters& parameters = {});
    ~DroneEntity();

    DroneEntity(const DroneEntity&) = delete;
    DroneEntity& operator=(const DroneEntity&) = delete;
    DroneEntity(DroneEntity&&) = delete;
    DroneEntity& operator=(DroneEntity&&) = delete;

    void setThrottle(double value);
    double throttle() const;

    void step(double dt);

    // Downward distance sensor reading in meters (ground clearance).
    double altitude() const;
    double verticalVelocity() const;
    double elapsedTime() const;

    const physics::RigidBodyState* body() const;
    const physics::CollisionShape& bodyShape() const;
    const ImuModel& imu() const;
    const DistanceSensorModel& distanceSensor() const;
    const QuadcopterForceModel& quadcopter() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::bridge
