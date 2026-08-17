#pragma once

#include "core/math/types.hpp"
#include "physics/collision_shape.hpp"
#include "physics/contact.hpp"

namespace srp::rendering
{

void drawCollisionShape(
    const srp::physics::CollisionShape& shape,
    const srp::math::Vec3& position,
    const srp::math::Quat& orientation);

void drawContactPoint(const srp::physics::ContactPoint& contact_point);

}  // namespace srp::rendering
