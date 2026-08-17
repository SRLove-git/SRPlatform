#include "bridge/arm_entity.hpp"

#include <algorithm>
#include <cmath>

#include <glm/gtc/quaternion.hpp>

namespace srp::bridge
{

namespace
{

double clampServo(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

}  // namespace

struct ArmEntity::Impl
{
    ArmParameters parameters;
    std::vector<double> joint_angles;
    std::vector<double> joint_targets;

    explicit Impl(const ArmParameters& values)
        : parameters(values)
    {
        if (parameters.link_lengths_m.empty())
        {
            parameters.link_lengths_m = {0.6, 0.5, 0.4};
        }
        parameters.joint_limits_rad.resize(parameters.link_lengths_m.size(), 2.5);
        parameters.joint_speed_rad_s.resize(parameters.link_lengths_m.size(), 1.5);

        joint_angles.assign(parameters.link_lengths_m.size(), 0.0);
        joint_targets.assign(parameters.link_lengths_m.size(), 0.0);
    }

    double cumulativeAngle(std::size_t link_index) const
    {
        double angle = 0.0;
        for (std::size_t i = 0; i <= link_index && i < joint_angles.size(); ++i)
        {
            angle += joint_angles[i];
        }
        return angle;
    }
};

ArmEntity::ArmEntity(const ArmParameters& parameters)
    : impl_(std::make_unique<Impl>(parameters))
{
}

ArmEntity::~ArmEntity() = default;

void ArmEntity::setServo(std::size_t joint_index, double value)
{
    if (joint_index >= impl_->joint_angles.size())
    {
        return;
    }
    setJointTarget(
        joint_index,
        clampServo(value) * impl_->parameters.joint_limits_rad[joint_index]);
}

void ArmEntity::setJointTarget(std::size_t joint_index, double radians)
{
    if (joint_index >= impl_->joint_angles.size())
    {
        return;
    }
    impl_->joint_targets[joint_index] = std::clamp(
        radians,
        -impl_->parameters.joint_limits_rad[joint_index],
        impl_->parameters.joint_limits_rad[joint_index]);
}

std::size_t ArmEntity::jointCount() const
{
    return impl_->joint_angles.size();
}

double ArmEntity::jointAngle(std::size_t joint_index) const
{
    if (joint_index >= impl_->joint_angles.size())
    {
        return 0.0;
    }
    return impl_->joint_angles[joint_index];
}

double ArmEntity::jointTarget(std::size_t joint_index) const
{
    if (joint_index >= impl_->joint_targets.size())
    {
        return 0.0;
    }
    return impl_->joint_targets[joint_index];
}

std::pair<math::Vec3, math::Quat> ArmEntity::linkTransform(
    std::size_t link_index) const
{
    if (link_index >= impl_->parameters.link_lengths_m.size())
    {
        return {basePosition(), math::Quat(1.0, 0.0, 0.0, 0.0)};
    }

    const double cumulative = impl_->cumulativeAngle(link_index);
    const math::Quat orientation = math::Quat(
        std::cos(cumulative * 0.5),
        0.0,
        0.0,
        std::sin(cumulative * 0.5));

    math::Vec3 start = basePosition();
    for (std::size_t i = 0; i < link_index; ++i)
    {
        const double angle = impl_->cumulativeAngle(i);
        start += math::Vec3(
            std::cos(angle) * impl_->parameters.link_lengths_m[i],
            std::sin(angle) * impl_->parameters.link_lengths_m[i],
            0.0);
    }

    const double length = impl_->parameters.link_lengths_m[link_index];
    const math::Vec3 center = start + math::Vec3(
        std::cos(cumulative) * length * 0.5,
        std::sin(cumulative) * length * 0.5,
        0.0);
    return {center, orientation};
}

math::Vec3 ArmEntity::endEffectorPosition() const
{
    math::Vec3 position = basePosition();
    for (std::size_t i = 0; i < impl_->parameters.link_lengths_m.size(); ++i)
    {
        const double angle = impl_->cumulativeAngle(i);
        position += math::Vec3(
            std::cos(angle) * impl_->parameters.link_lengths_m[i],
            std::sin(angle) * impl_->parameters.link_lengths_m[i],
            0.0);
    }
    return position;
}

math::Vec3 ArmEntity::basePosition() const
{
    return math::Vec3(0.0, impl_->parameters.base_height_m, 0.0);
}

void ArmEntity::step(double dt)
{
    for (std::size_t i = 0; i < impl_->joint_angles.size(); ++i)
    {
        const double error = impl_->joint_targets[i] - impl_->joint_angles[i];
        const double max_step =
            impl_->parameters.joint_speed_rad_s[i] * dt;
        const double correction = std::clamp(error, -max_step, max_step);
        impl_->joint_angles[i] += correction;
    }
}

const char* ArmEntity::kind() const
{
    return "arm";
}

}  // namespace srp::bridge
