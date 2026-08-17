#pragma once

#include "physics/rigid_body.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace srp::physics
{

class PhysicsWorld
{
public:
    PhysicsWorld();

    BodyId createBody(const RigidBodyState& state = {});

    RigidBodyState* body(BodyId id);
    const RigidBodyState* body(BodyId id) const;

    void setGravity(const math::Vec3& gravity);
    math::Vec3 gravity() const;

    void step(double dt);

private:
    std::vector<RigidBodyState> bodies_;
    std::unordered_map<BodyId, std::size_t> body_indices_;
    BodyId next_body_id_{kInvalidBodyId + 1};
    math::Vec3 gravity_{0.0, -9.81, 0.0};
};

}  // namespace srp::physics
