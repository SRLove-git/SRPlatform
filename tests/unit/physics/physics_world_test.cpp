#include "physics/physics_world.hpp"

#include <gtest/gtest.h>

TEST(PhysicsWorld, GravityMovesDynamicBody)
{
    srp::physics::PhysicsWorld world;
    const srp::physics::BodyId id = world.createBody();

    srp::physics::RigidBodyState* state = world.body(id);
    ASSERT_NE(state, nullptr);

    world.setGravity(srp::math::Vec3(0.0, -9.81, 0.0));
    world.step(1.0);

    EXPECT_NEAR(state->linear_velocity.y, -9.81, 1e-12);
    EXPECT_NEAR(state->position.y, -9.81, 1e-12);
}

TEST(PhysicsWorld, StaticBodyIgnoresGravity)
{
    srp::physics::RigidBodyState initial_state;
    initial_state.type = srp::physics::RigidBodyType::kStatic;

    srp::physics::PhysicsWorld world;
    const srp::physics::BodyId id = world.createBody(initial_state);

    world.step(1.0);

    const srp::physics::RigidBodyState* state = world.body(id);
    ASSERT_NE(state, nullptr);
    EXPECT_DOUBLE_EQ(state->position.y, 0.0);
    EXPECT_DOUBLE_EQ(state->linear_velocity.y, 0.0);
}

TEST(PhysicsWorld, KinematicBodyIgnoresGravity)
{
    srp::physics::RigidBodyState initial_state;
    initial_state.type = srp::physics::RigidBodyType::kKinematic;

    srp::physics::PhysicsWorld world;
    const srp::physics::BodyId id = world.createBody(initial_state);

    world.step(1.0);

    const srp::physics::RigidBodyState* state = world.body(id);
    ASSERT_NE(state, nullptr);
    EXPECT_DOUBLE_EQ(state->position.y, 0.0);
    EXPECT_DOUBLE_EQ(state->linear_velocity.y, 0.0);
}
