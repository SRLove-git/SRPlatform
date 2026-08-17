#pragma once

#include "physics/collision_shape.hpp"
#include "physics/contact.hpp"
#include "physics/joint.hpp"
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

    BodyId createBody(
        const RigidBodyState& state = {},
        CollisionShape shape = BoxShape{});

    JointId createJoint(const JointDefinition& definition);

    RigidBodyState* body(BodyId id);
    const RigidBodyState* body(BodyId id) const;

    CollisionShape* shape(BodyId id);
    const CollisionShape* shape(BodyId id) const;

    const std::vector<BodyId>& bodyIds() const;

    Joint* joint(JointId id);
    const Joint* joint(JointId id) const;

    const std::vector<Contact>& contacts() const;

    void setGravity(const math::Vec3& gravity);
    math::Vec3 gravity() const;

    // Queues a world-frame force or torque on a body. Accumulated forces
    // are integrated and cleared on the next step().
    void applyForce(BodyId id, const math::Vec3& force);
    void applyTorque(BodyId id, const math::Vec3& torque);

    void step(double dt);

private:
    std::vector<BodyId> body_ids_;
    std::vector<RigidBodyState> bodies_;
    std::vector<CollisionShape> shapes_;
    std::vector<math::Vec3> force_accumulators_;
    std::vector<math::Vec3> torque_accumulators_;
    std::vector<Joint> joints_;
    std::vector<Contact> last_contacts_;
    std::unordered_map<BodyId, std::size_t> body_indices_;
    std::unordered_map<JointId, std::size_t> joint_indices_;
    BodyId next_body_id_{kInvalidBodyId + 1};
    JointId next_joint_id_{kInvalidJointId + 1};
    math::Vec3 gravity_{0.0, -9.81, 0.0};

    std::vector<Contact> generateContacts() const;
    void solveContacts(const std::vector<Contact>& contacts);
    void solveJoints(double dt);
    void solveWheelJoint(const Joint& joint, double dt);
};

}  // namespace srp::physics
