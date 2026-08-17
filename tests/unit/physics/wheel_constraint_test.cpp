#include "physics/physics_world.hpp"

#include <gtest/gtest.h>

namespace
{

srp::physics::BodyId createGround(srp::physics::PhysicsWorld& world)
{
    srp::physics::RigidBodyState state;
    state.type = srp::physics::RigidBodyType::kStatic;
    return world.createBody(state, srp::physics::PlaneShape{});
}

srp::physics::BodyId createWheel(srp::physics::PhysicsWorld& world)
{
    srp::physics::RigidBodyState state;
    state.type = srp::physics::RigidBodyType::kDynamic;
    state.mass = 1.0;
    state.position = srp::math::Vec3(0.0, 0.25, 0.0);
    state.inertia_local = srp::math::Mat3(0.036);
    return world.createBody(state, srp::physics::SphereShape{0.3});
}

srp::physics::BodyId createChassis(srp::physics::PhysicsWorld& world)
{
    srp::physics::RigidBodyState state;
    state.type = srp::physics::RigidBodyType::kDynamic;
    state.mass = 1.0;
    state.position = srp::math::Vec3(0.0, 0.6, 0.0);
    return world.createBody(state, srp::physics::BoxShape{});
}

srp::physics::JointDefinition wheelJoint(
    srp::physics::BodyId chassis,
    srp::physics::BodyId wheel,
    double drive_torque)
{
    srp::physics::JointDefinition joint;
    joint.type = srp::physics::JointType::kWheel;
    joint.body_a = chassis;
    joint.body_b = wheel;
    joint.anchor_local_a = srp::math::Vec3(0.0, -0.3, 0.0);
    joint.anchor_local_b = srp::math::Vec3(0.0);
    joint.axis_local_a = srp::math::Vec3(0.0, 0.0, 1.0);
    joint.axis_local_b = srp::math::Vec3(0.0, 0.0, 1.0);
    joint.wheel_radius = 0.3;
    joint.drive_torque = drive_torque;
    return joint;
}

}  // namespace

TEST(PhysicsWheelConstraint, DriveTorquePushesChassisForward)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0));

    createGround(world);
    const srp::physics::BodyId wheel = createWheel(world);
    const srp::physics::BodyId chassis = createChassis(world);

    EXPECT_NE(world.createJoint(wheelJoint(chassis, wheel, 0.1)), srp::physics::kInvalidJointId);

    for (int step = 0; step < 120; ++step)
    {
        world.step(1.0 / 60.0);
    }

    const srp::physics::RigidBodyState* wheel_state = world.body(wheel);
    const srp::physics::RigidBodyState* chassis_state = world.body(chassis);
    ASSERT_NE(wheel_state, nullptr);
    ASSERT_NE(chassis_state, nullptr);

    EXPECT_GT(chassis_state->position.x, 0.0);
    EXPECT_GT(wheel_state->angular_velocity.z, 0.0);
    EXPECT_NEAR(
        wheel_state->angular_velocity.z * 0.3,
        chassis_state->linear_velocity.x,
        1e-9);
}

TEST(PhysicsWheelConstraint, InitialSpinMovesChassisWithoutDriveTorque)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0));

    createGround(world);
    const srp::physics::BodyId wheel = createWheel(world);
    const srp::physics::BodyId chassis = createChassis(world);

    srp::physics::RigidBodyState* wheel_state = world.body(wheel);
    ASSERT_NE(wheel_state, nullptr);
    wheel_state->angular_velocity = srp::math::Vec3(0.0, 0.0, 10.0);

    EXPECT_NE(world.createJoint(wheelJoint(chassis, wheel, 0.0)), srp::physics::kInvalidJointId);

    const srp::physics::RigidBodyState* chassis_state = world.body(chassis);
    ASSERT_NE(chassis_state, nullptr);

    world.step(1.0 / 60.0);
    EXPECT_GT(wheel_state->angular_velocity.z, 0.0);
    EXPECT_GT(chassis_state->linear_velocity.x, 0.0);
    EXPECT_NEAR(
        wheel_state->angular_velocity.z * 0.3,
        chassis_state->linear_velocity.x,
        1e-9);

    world.step(1.0 / 60.0);
    EXPECT_GT(chassis_state->position.x, 0.0);
}

TEST(PhysicsWheelConstraint, IgnoresInvalidWheelRadius)
{
    srp::physics::PhysicsWorld world;
    createGround(world);
    const srp::physics::BodyId wheel = createWheel(world);
    const srp::physics::BodyId chassis = createChassis(world);

    srp::physics::JointDefinition joint = wheelJoint(chassis, wheel, 0.0);
    joint.wheel_radius = 0.0;

    EXPECT_NE(world.createJoint(joint), srp::physics::kInvalidJointId);

    srp::physics::RigidBodyState* chassis_state = world.body(chassis);
    ASSERT_NE(chassis_state, nullptr);
    const double initial_x = chassis_state->position.x;

    for (int step = 0; step < 10; ++step)
    {
        world.step(1.0 / 60.0);
    }

    EXPECT_DOUBLE_EQ(chassis_state->position.x, initial_x);
}
