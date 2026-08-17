#pragma once

#include "core/math/types.hpp"
#include "physics/rigid_body.hpp"

namespace srp::physics
{
class PhysicsWorld;
}

namespace srp::bridge
{

// Distance sensor parameters. All values use SI units.
struct DistanceSensorParameters
{
    // Readings beyond this range are reported as "no detection" and the
    // sensor returns max_range_m.
    double max_range_m{4.0};

    // Beam direction in the sensor's local frame. The default points down,
    // matching a drone altimeter; rotate the sensor pose to aim elsewhere.
    math::Vec3 beam_axis_local{0.0, -1.0, 0.0};
};

// Ideal single-beam distance sensor (ultrasonic / ToF style).
//
// update() casts one ray from the sensor pose along the beam axis against
// every body in the physics world and reports the nearest hit within
// max_range_m. Boxes, spheres, planes, and cylinders use exact ray
// intersection; convex hulls use their bounding sphere as an approximation.
class DistanceSensorModel
{
public:
    explicit DistanceSensorModel(
        const DistanceSensorParameters& parameters = {});

    void setPose(const math::Vec3& position, const math::Quat& orientation);
    math::Vec3 origin() const;
    math::Vec3 beamDirectionWorld() const;

    // Bodies with this id are skipped during update(). Use kInvalidBodyId
    // (the default) to disable ignoring.
    void setIgnoredBody(physics::BodyId id);

    // Re-measure against the current physics world.
    void update(const physics::PhysicsWorld& world);

    // True when an object was found within max_range_m.
    bool detected() const;

    // Distance to the nearest object in meters. When nothing is detected
    // the reading is clamped to max_range_m, like a real rangefinder.
    double distance() const;

    void reset();

private:
    DistanceSensorParameters parameters_;
    math::Vec3 position_{0.0};
    math::Quat orientation_{1.0, 0.0, 0.0, 0.0};
    math::Vec3 beam_direction_world_{0.0, -1.0, 0.0};
    bool detected_{false};
    double distance_m_{0.0};
    physics::BodyId ignored_body_{physics::kInvalidBodyId};
};

}  // namespace srp::bridge
