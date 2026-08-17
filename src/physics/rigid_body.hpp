#pragma once

#include "core/math/si_units.hpp"
#include "core/math/types.hpp"

#include <cstdint>

namespace srp::physics
{

using BodyId = std::uint32_t;

constexpr BodyId kInvalidBodyId = 0;

enum class RigidBodyType
{
    kStatic,
    kDynamic,
    kKinematic
};

struct RigidBodyState
{
    math::Vec3 position{0.0};
    math::Quat orientation{1.0, 0.0, 0.0, 0.0};
    math::Vec3 linear_velocity{0.0};
    math::Vec3 angular_velocity{0.0};
    math::Mass mass{1.0};
    math::Mat3 inertia_local{1.0};
    math::Scalar restitution{0.0};
    math::Scalar friction{0.5};
    RigidBodyType type{RigidBodyType::kDynamic};
};

void normalizeOrientation(RigidBodyState& state);
math::Vec3 localToWorld(const RigidBodyState& state, const math::Vec3& local_point);
math::Vec3 worldToLocal(const RigidBodyState& state, const math::Vec3& world_point);

}  // namespace srp::physics
