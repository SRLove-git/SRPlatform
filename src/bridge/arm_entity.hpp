#pragma once

#include "bridge/entity.hpp"
#include "core/math/types.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace srp::bridge
{

struct ArmParameters
{
    // Base pivot height above the ground.
    double base_height_m{0.6};

    // Link lengths from the pivot outward.
    std::vector<double> link_lengths_m{0.6, 0.5, 0.4};

    // Servo range per joint in radians (the actuator value -1..1 maps to
    // -limit..limit around the zero pose).
    std::vector<double> joint_limits_rad{2.5, 2.5, 2.5};

    // Maximum joint speed in rad/s.
    std::vector<double> joint_speed_rad_s{1.5, 1.5, 1.5};
};

// A kinematic teaching robotic arm. Joint angles are driven by actuator
// commands (servo value -1..1 maps to the joint limit); links are rendered
// from forward kinematics, so no physics solver is involved.
class ArmEntity : public IEntity
{
public:
    explicit ArmEntity(const ArmParameters& parameters = {});
    ~ArmEntity();

    ArmEntity(const ArmEntity&) = delete;
    ArmEntity& operator=(const ArmEntity&) = delete;

    // value is clamped to [-1, 1] and mapped to the joint limit.
    void setServo(std::size_t joint_index, double value);
    void setJointTarget(std::size_t joint_index, double radians);

    std::size_t jointCount() const;
    double jointAngle(std::size_t joint_index) const;
    double jointTarget(std::size_t joint_index) const;

    // World transform (position, orientation) of link i. Link 0 starts at the
    // base pivot; the arm lies in the XY plane with X pointing forward.
    std::pair<math::Vec3, math::Quat> linkTransform(std::size_t link_index) const;

    math::Vec3 endEffectorPosition() const;
    math::Vec3 basePosition() const;

    void step(double dt) override;
    const char* kind() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::bridge
