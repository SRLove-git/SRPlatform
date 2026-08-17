#include "physics/rigid_body.hpp"

#include <glm/gtc/quaternion.hpp>

namespace srp::physics
{

void normalizeOrientation(RigidBodyState& state)
{
    state.orientation = glm::normalize(state.orientation);
}

math::Vec3 localToWorld(const RigidBodyState& state, const math::Vec3& local_point)
{
    return state.position + (state.orientation * local_point);
}

math::Vec3 worldToLocal(const RigidBodyState& state, const math::Vec3& world_point)
{
    return glm::inverse(state.orientation) * (world_point - state.position);
}

}  // namespace srp::physics
