#pragma once

#include "core/math/types.hpp"
#include "physics/collision_shape.hpp"

namespace srp::physics
{

struct CollisionResult
{
    bool collided{false};
    math::Vec3 point{0.0};
    math::Vec3 normal{0.0};
    math::Scalar penetration{0.0};
};

bool collide(
    const CollisionShape& shape_a,
    const math::Vec3& position_a,
    const math::Quat& orientation_a,
    const CollisionShape& shape_b,
    const math::Vec3& position_b,
    const math::Quat& orientation_b,
    CollisionResult& result);

}  // namespace srp::physics
