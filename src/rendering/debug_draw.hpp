#pragma once

#include "core/math/types.hpp"
#include "mod/gltf_model.hpp"
#include "physics/collision_shape.hpp"
#include "physics/contact.hpp"

namespace srp::rendering
{

void drawCollisionShape(
    const srp::physics::CollisionShape& shape,
    const srp::math::Vec3& position,
    const srp::math::Quat& orientation);

void drawContactPoint(const srp::physics::ContactPoint& contact_point);

void drawMesh(
    const srp::mod::GltfMesh& mesh,
    const srp::math::Vec3& position,
    const srp::math::Quat& orientation,
    double scale = 1.0);

}  // namespace srp::rendering
