#include "physics/physics_world.hpp"

#include <algorithm>

#include <gtest/gtest.h>

namespace srp::physics
{
namespace
{

TEST(BodyRemovalTest, RemovesBodyAndUpdatesBodyIds)
{
    PhysicsWorld world;
    const BodyId first = world.createBody();
    const BodyId second = world.createBody();
    const BodyId third = world.createBody();

    EXPECT_TRUE(world.removeBody(second));
    EXPECT_EQ(world.body(second), nullptr);
    EXPECT_NE(world.body(first), nullptr);
    EXPECT_NE(world.body(third), nullptr);

    const std::vector<BodyId>& ids = world.bodyIds();
    EXPECT_EQ(ids.size(), 2u);
    EXPECT_NE(std::find(ids.begin(), ids.end(), first), ids.end());
    EXPECT_NE(std::find(ids.begin(), ids.end(), third), ids.end());

    EXPECT_FALSE(world.removeBody(second));
}

TEST(BodyRemovalTest, RemoveBodyDropsReferencingJoints)
{
    PhysicsWorld world;
    const BodyId body_a = world.createBody();
    const BodyId body_b = world.createBody();

    JointDefinition definition;
    definition.type = JointType::kFixed;
    definition.body_a = body_a;
    definition.body_b = body_b;
    const JointId joint_id = world.createJoint(definition);
    ASSERT_NE(joint_id, kInvalidJointId);

    EXPECT_TRUE(world.removeBody(body_b));
    EXPECT_EQ(world.joint(joint_id), nullptr);
    EXPECT_NE(world.body(body_a), nullptr);
}

TEST(BodyRemovalTest, RemoveJointKeepsOtherJoints)
{
    PhysicsWorld world;
    const BodyId body_a = world.createBody();
    const BodyId body_b = world.createBody();

    JointDefinition first_definition;
    first_definition.type = JointType::kHinge;
    first_definition.body_a = body_a;
    first_definition.body_b = body_b;
    const JointId first = world.createJoint(first_definition);

    JointDefinition second_definition;
    second_definition.type = JointType::kHinge;
    second_definition.body_a = body_a;
    second_definition.body_b = body_b;
    const JointId second = world.createJoint(second_definition);

    EXPECT_TRUE(world.removeJoint(first));
    EXPECT_EQ(world.joint(first), nullptr);
    EXPECT_NE(world.joint(second), nullptr);
}

TEST(BodyRemovalTest, RemovalSurvivesStepping)
{
    PhysicsWorld world;
    const BodyId dropped = world.createBody();
    RigidBodyState* dropped_state = world.body(dropped);
    dropped_state->position = math::Vec3(0.0, 2.0, 0.0);
    world.createBody();

    world.step(1.0 / 60.0);
    EXPECT_TRUE(world.removeBody(dropped));
    world.step(1.0 / 60.0);
    EXPECT_EQ(world.bodyIds().size(), 1u);
    EXPECT_EQ(world.contacts().size(), 0u);
}

}  // namespace
}  // namespace srp::physics
