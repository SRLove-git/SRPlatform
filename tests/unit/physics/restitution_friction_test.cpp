#include "physics/physics_world.hpp"

#include <gtest/gtest.h>

TEST(PhysicsRestitution, SphereBouncesOffPlane)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0, 0.0, 0.0));

    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;

    srp::physics::PlaneShape plane;
    plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    world.createBody(ground_state, plane);

    srp::physics::RigidBodyState sphere_state;
    sphere_state.type = srp::physics::RigidBodyType::kDynamic;
    sphere_state.mass = 1.0;
    sphere_state.position = srp::math::Vec3(0.0, 0.55, 0.0);
    sphere_state.linear_velocity = srp::math::Vec3(0.0, -5.0, 0.0);
    sphere_state.restitution = 0.8;

    srp::physics::SphereShape sphere;
    sphere.radius = 0.5;
    const srp::physics::BodyId sphere_id = world.createBody(sphere_state, sphere);

    world.step(0.1);

    const srp::physics::RigidBodyState* body = world.body(sphere_id);
    ASSERT_NE(body, nullptr);
    EXPECT_GT(body->linear_velocity.y, 0.0);
}

TEST(PhysicsFriction, SphereHorizontalVelocityDecreases)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0, -9.81, 0.0));

    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;

    srp::physics::PlaneShape plane;
    plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    world.createBody(ground_state, plane);

    srp::physics::RigidBodyState sphere_state;
    sphere_state.type = srp::physics::RigidBodyType::kDynamic;
    sphere_state.mass = 1.0;
    sphere_state.position = srp::math::Vec3(0.0, 0.49, 0.0);
    sphere_state.linear_velocity = srp::math::Vec3(2.0, 0.0, 0.0);
    sphere_state.friction = 0.5;

    srp::physics::SphereShape sphere;
    sphere.radius = 0.5;
    const srp::physics::BodyId sphere_id = world.createBody(sphere_state, sphere);

    world.step(1.0 / 60.0);

    const srp::physics::RigidBodyState* body = world.body(sphere_id);
    ASSERT_NE(body, nullptr);
    EXPECT_LT(body->linear_velocity.x, 2.0);
}
