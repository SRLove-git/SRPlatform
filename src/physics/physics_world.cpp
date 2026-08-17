#include "physics/physics_world.hpp"

#include <glm/gtc/quaternion.hpp>

namespace srp::physics
{

PhysicsWorld::PhysicsWorld() = default;

BodyId PhysicsWorld::createBody(const RigidBodyState& state)
{
    const BodyId id = next_body_id_++;
    body_indices_[id] = bodies_.size();
    bodies_.push_back(state);
    return id;
}

RigidBodyState* PhysicsWorld::body(BodyId id)
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return nullptr;
    }

    return &bodies_[it->second];
}

const RigidBodyState* PhysicsWorld::body(BodyId id) const
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return nullptr;
    }

    return &bodies_[it->second];
}

void PhysicsWorld::setGravity(const math::Vec3& gravity)
{
    gravity_ = gravity;
}

math::Vec3 PhysicsWorld::gravity() const
{
    return gravity_;
}

void PhysicsWorld::step(double dt)
{
    for (RigidBodyState& state : bodies_)
    {
        if (state.type != RigidBodyType::kDynamic)
        {
            continue;
        }

        state.linear_velocity += gravity_ * dt;
        state.position += state.linear_velocity * dt;

        const math::Quat angular_velocity_quaternion(
            0.0,
            state.angular_velocity.x,
            state.angular_velocity.y,
            state.angular_velocity.z);

        state.orientation += 0.5 * dt * (angular_velocity_quaternion * state.orientation);
        normalizeOrientation(state);
    }
}

}  // namespace srp::physics
