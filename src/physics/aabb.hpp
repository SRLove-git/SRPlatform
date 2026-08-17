#pragma once

#include "core/math/types.hpp"
#include "physics/collision_shape.hpp"

namespace srp::physics
{

struct Aabb
{
    math::Vec3 min{0.0};
    math::Vec3 max{0.0};
    bool is_finite{true};
};

Aabb computeAabb(
    const CollisionShape& shape,
    const math::Vec3& position,
    const math::Quat& orientation);

bool overlaps(const Aabb& lhs, const Aabb& rhs);

}  // namespace srp::physics
