#include "bridge/imu_model.hpp"

#include <algorithm>
#include <cmath>

#include <glm/gtc/quaternion.hpp>

namespace srp::bridge
{
namespace
{

// World-frame gravity vector for the accelerometer convention.
constexpr math::Vec3 kGravity{0.0, -9.81, 0.0};

math::Vec3 quaternionToAttitude(const math::Quat& orientation)
{
    // The body-to-world rotation matrix is R = Rx(roll) * Rz(pitch) *
    // Ry(yaw). With glm's column-major indexing:
    //   pitch = asin(-R[1][0]), yaw = atan2(R[2][0], R[0][0]),
    //   roll  = atan2(R[1][2], R[1][1]).
    const math::Mat3 rotation = glm::mat3_cast(orientation);

    const double pitch_value =
        std::asin(std::clamp(-rotation[1][0], -1.0, 1.0));
    const double yaw_value = std::atan2(rotation[2][0], rotation[0][0]);
    const double roll_value = std::atan2(rotation[1][2], rotation[1][1]);

    return math::Vec3(roll_value, pitch_value, yaw_value);
}

}  // namespace

ImuModel::ImuModel(const ImuParameters& parameters)
    : parameters_(parameters)
{
}

void ImuModel::update(
    const physics::RigidBodyState& state,
    const math::Vec3& world_linear_acceleration)
{
    const math::Quat orientation = glm::normalize(state.orientation);
    const math::Quat inverse_orientation = glm::inverse(orientation);

    orientation_ = orientation;
    attitude_ = quaternionToAttitude(orientation);

    angular_velocity_body_ =
        inverse_orientation * state.angular_velocity +
        parameters_.gyro_bias_rad_s;

    const math::Vec3 specific_force_world =
        world_linear_acceleration - kGravity;
    acceleration_body_ =
        inverse_orientation * specific_force_world +
        parameters_.accelerometer_bias_m_s2;
}

math::Vec3 ImuModel::attitude() const
{
    return attitude_;
}

double ImuModel::roll() const
{
    return attitude_.x;
}

double ImuModel::pitch() const
{
    return attitude_.y;
}

double ImuModel::yaw() const
{
    return attitude_.z;
}

math::Quat ImuModel::orientation() const
{
    return orientation_;
}

math::Vec3 ImuModel::angularVelocity() const
{
    return angular_velocity_body_;
}

math::Vec3 ImuModel::acceleration() const
{
    return acceleration_body_;
}

void ImuModel::reset()
{
    attitude_ = math::Vec3(0.0);
    orientation_ = math::Quat(1.0, 0.0, 0.0, 0.0);
    angular_velocity_body_ = math::Vec3(0.0);
    acceleration_body_ = math::Vec3(0.0);
}

}  // namespace srp::bridge
