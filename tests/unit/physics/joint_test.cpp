#include "physics/joint.hpp"
#include "physics/physics_world.hpp"

#include <gtest/gtest.h>

namespace
{

srp::physics::BodyId createStaticBody(srp::physics::PhysicsWorld& world)
{
    srp::physics::RigidBodyState state;
    state.type = srp::physics::RigidBodyType::kStatic;
    return world.createBody(state, srp::physics::BoxShape{});
}

}  // namespace

TEST(PhysicsJoint, FixedJointHoldsBodyAgainstGravity)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0, -9.81, 0.0));

    const srp::physics::BodyId static_id = createStaticBody(world);

    srp::physics::RigidBodyState dynamic_state;
    dynamic_state.type = srp::physics::RigidBodyType::kDynamic;
    dynamic_state.mass = 1.0;
    dynamic_state.position = srp::math::Vec3(0.0, -1.0, 0.0);
    const srp::physics::BodyId dynamic_id = world.createBody(dynamic_state, srp::physics::BoxShape{});

    srp::physics::JointDefinition joint;
    joint.type = srp::physics::JointType::kFixed;
    joint.body_a = static_id;
    joint.body_b = dynamic_id;
    joint.anchor_local_a = srp::math::Vec3(0.0);
    joint.anchor_local_b = srp::math::Vec3(0.0);
    EXPECT_NE(world.createJoint(joint), srp::physics::kInvalidJointId);

    for (int i = 0; i < 600; ++i)
    {
        world.step(1.0 / 60.0);
    }

    const srp::physics::RigidBodyState* body = world.body(dynamic_id);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->position.y, 0.0, 0.05);
}

TEST(PhysicsJoint, HingeJointPreventsAnchorSeparation)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0, -9.81, 0.0));

    const srp::physics::BodyId static_id = createStaticBody(world);

    srp::physics::RigidBodyState dynamic_state;
    dynamic_state.type = srp::physics::RigidBodyType::kDynamic;
    dynamic_state.mass = 1.0;
    dynamic_state.position = srp::math::Vec3(0.0, -1.0, 0.0);
    const srp::physics::BodyId dynamic_id = world.createBody(dynamic_state, srp::physics::BoxShape{});

    srp::physics::JointDefinition joint;
    joint.type = srp::physics::JointType::kHinge;
    joint.body_a = static_id;
    joint.body_b = dynamic_id;
    joint.anchor_local_a = srp::math::Vec3(0.0);
    joint.anchor_local_b = srp::math::Vec3(0.0);
    EXPECT_NE(world.createJoint(joint), srp::physics::kInvalidJointId);

    for (int i = 0; i < 600; ++i)
    {
        world.step(1.0 / 60.0);
    }

    const srp::physics::RigidBodyState* body = world.body(dynamic_id);
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(body->position.y, 0.0, 0.05);
}
