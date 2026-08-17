#pragma once

#include "core/math/types.hpp"
#include "physics/rigid_body.hpp"

namespace srp::bridge
{

// Sensor calibration parameters. All values use SI units: rad/s for the
// gyroscope and m/s^2 for the accelerometer.
struct ImuParameters
{
    math::Vec3 gyro_bias_rad_s{0.0};
    math::Vec3 accelerometer_bias_m_s2{0.0};
};

// Ideal inertial measurement unit for a rigid body.
//
// update() consumes the body state plus its world-frame center-of-mass
// linear acceleration and produces:
//
//   - attitude: roll/pitch/yaw in radians, in the same right-handed body
//     frame used by the quadcopter model (X forward, Y up, Z left):
//       roll  = rotation about the forward X axis
//       pitch = rotation about the lateral Z axis
//       yaw   = rotation about the vertical Y axis
//   - angular velocity: gyro readings in the body frame, rad/s
//   - acceleration: accelerometer readings in the body frame, m/s^2
//
// The accelerometer reports specific force: a_world - g expressed in body
// coordinates, so a level body at rest reads +9.81 m/s^2 along its up axis.
class ImuModel
{
public:
    explicit ImuModel(const ImuParameters& parameters = {});

    // Feed the current body state and the body's world-frame linear
    // acceleration of its center of mass. The previous readings are
    // replaced by the new ones.
    void update(
        const physics::RigidBodyState& state,
        const math::Vec3& world_linear_acceleration);

    // Attitude angles in radians: x = roll, y = pitch, z = yaw.
    math::Vec3 attitude() const;
    double roll() const;
    double pitch() const;
    double yaw() const;

    // The body orientation from the last update, normalized.
    math::Quat orientation() const;

    // Body-frame gyro readings in rad/s, including bias.
    math::Vec3 angularVelocity() const;

    // Body-frame accelerometer readings in m/s^2, including bias.
    math::Vec3 acceleration() const;

    void reset();

private:
    ImuParameters parameters_;
    math::Vec3 attitude_{0.0};
    math::Quat orientation_{1.0, 0.0, 0.0, 0.0};
    math::Vec3 angular_velocity_body_{0.0};
    math::Vec3 acceleration_body_{0.0};
};

}  // namespace srp::bridge
