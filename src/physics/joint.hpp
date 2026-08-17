#pragma once

#include "core/math/types.hpp"
#include "physics/rigid_body.hpp"

#include <cstdint>

namespace srp::physics
{

using JointId = std::uint32_t;

constexpr JointId kInvalidJointId = 0;

enum class JointType
{
    kFixed,
    kHinge
};

struct JointDefinition
{
    JointType type{JointType::kFixed};
    BodyId body_a{kInvalidBodyId};
    BodyId body_b{kInvalidBodyId};
    math::Vec3 anchor_local_a{0.0};
    math::Vec3 anchor_local_b{0.0};
    math::Vec3 axis_local_a{0.0, 1.0, 0.0};
    math::Vec3 axis_local_b{0.0, 1.0, 0.0};
};

using Joint = JointDefinition;

}  // namespace srp::physics
