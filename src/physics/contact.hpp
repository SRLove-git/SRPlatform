#pragma once

#include "core/math/types.hpp"
#include "physics/collision_shape.hpp"
#include "physics/narrowphase.hpp"
#include "physics/rigid_body.hpp"

#include <optional>

namespace srp::physics
{

struct ContactPoint
{
    math::Vec3 point{0.0};
    math::Vec3 normal{0.0};
    math::Scalar penetration{0.0};
    // Total normal-impulse magnitude applied by the contact solver across
    // the last step. Force is approximately impulse / dt.
    math::Scalar normal_impulse{0.0};
};

struct Contact
{
    BodyId body_a{kInvalidBodyId};
    BodyId body_b{kInvalidBodyId};
    ContactPoint point;
};

inline std::optional<Contact> generateContact(
    BodyId body_a,
    const CollisionShape& shape_a,
    const math::Vec3& position_a,
    const math::Quat& orientation_a,
    BodyId body_b,
    const CollisionShape& shape_b,
    const math::Vec3& position_b,
    const math::Quat& orientation_b)
{
    CollisionResult collision_result;
    if (!collide(
            shape_a,
            position_a,
            orientation_a,
            shape_b,
            position_b,
            orientation_b,
            collision_result))
    {
        return std::nullopt;
    }

    return Contact{
        body_a,
        body_b,
        {
            collision_result.point,
            collision_result.normal,
            collision_result.penetration}};
}

}  // namespace srp::physics
