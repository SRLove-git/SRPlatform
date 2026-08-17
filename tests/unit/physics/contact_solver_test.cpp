#include "physics/physics_world.hpp"

#include <gtest/gtest.h>

TEST(PhysicsContactSolver, SphereComesToRestOnPlane)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0, -9.81, 0.0));

    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;

    srp::physics::PlaneShape plane;
    plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    plane.offset = 0.0;
    world.createBody(ground_state, plane);

    srp::physics::RigidBodyState sphere_state;
    sphere_state.type = srp::physics::RigidBodyType::kDynamic;
    sphere_state.mass = 1.0;
    sphere_state.position = srp::math::Vec3(0.0, 2.0, 0.0);

    srp::physics::SphereShape sphere;
    sphere.radius = 0.5;
    const srp::physics::BodyId sphere_id = world.createBody(sphere_state, sphere);

    for (int i = 0; i < 600; ++i)
    {
        world.step(1.0 / 60.0);
    }

    const srp::physics::RigidBodyState* body = world.body(sphere_id);
    ASSERT_NE(body, nullptr);

    EXPECT_GE(body->position.y, 0.48);
    EXPECT_LE(body->position.y, 0.52);
    EXPECT_NEAR(body->linear_velocity.y, 0.0, 0.1);
}

TEST(PhysicsContactSolver, DynamicBoxDoesNotPenetrateStaticBox)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0, -9.81, 0.0));

    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;
    ground_state.position = srp::math::Vec3(0.0, -1.0, 0.0);

    srp::physics::BoxShape ground_box;
    ground_box.half_extents = srp::math::Vec3(2.0, 0.5, 2.0);
    world.createBody(ground_state, ground_box);

    srp::physics::RigidBodyState box_state;
    box_state.type = srp::physics::RigidBodyType::kDynamic;
    box_state.mass = 1.0;
    box_state.position = srp::math::Vec3(0.0, 1.0, 0.0);

    srp::physics::BoxShape box;
    box.half_extents = srp::math::Vec3(0.5);
    const srp::physics::BodyId box_id = world.createBody(box_state, box);

    for (int i = 0; i < 600; ++i)
    {
        world.step(1.0 / 60.0);
    }

    const srp::physics::RigidBodyState* body = world.body(box_id);
    ASSERT_NE(body, nullptr);

    EXPECT_GE(body->position.y, -0.02);
    EXPECT_LE(body->position.y, 0.02);
}
