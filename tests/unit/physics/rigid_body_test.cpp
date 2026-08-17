#include "physics/rigid_body.hpp"

#include <glm/gtc/quaternion.hpp>

#include <gtest/gtest.h>

TEST(PhysicsRigidBody, DefaultsAreSane)
{
    srp::physics::RigidBodyState state;

    EXPECT_DOUBLE_EQ(state.position.x, 0.0);
    EXPECT_DOUBLE_EQ(state.position.y, 0.0);
    EXPECT_DOUBLE_EQ(state.position.z, 0.0);
    EXPECT_DOUBLE_EQ(state.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(state.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(state.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(state.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(state.linear_velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(state.linear_velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(state.linear_velocity.z, 0.0);
    EXPECT_DOUBLE_EQ(state.angular_velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(state.angular_velocity.y, 0.0);
    EXPECT_DOUBLE_EQ(state.angular_velocity.z, 0.0);
    EXPECT_DOUBLE_EQ(state.mass, 1.0);
    EXPECT_DOUBLE_EQ(state.inertia_local[0][0], 1.0);
    EXPECT_DOUBLE_EQ(state.inertia_local[1][1], 1.0);
    EXPECT_DOUBLE_EQ(state.inertia_local[2][2], 1.0);
}

TEST(PhysicsRigidBody, NormalizesOrientation)
{
    srp::physics::RigidBodyState state;
    state.orientation = srp::math::Quat(2.0, 0.0, 0.0, 0.0);

    srp::physics::normalizeOrientation(state);

    EXPECT_NEAR(glm::length(state.orientation), 1.0, 1e-12);
}

TEST(PhysicsRigidBody, LocalAndWorldConversionRoundTrip)
{
    srp::physics::RigidBodyState state;
    state.position = srp::math::Vec3(1.0, 2.0, 3.0);
    state.orientation = glm::angleAxis(
        srp::math::radians(90.0),
        srp::math::Vec3(0.0, 1.0, 0.0));
    srp::physics::normalizeOrientation(state);

    const srp::math::Vec3 local_point{1.0, 0.5, -0.25};
    const srp::math::Vec3 world_point = srp::physics::localToWorld(state, local_point);
    const srp::math::Vec3 round_trip = srp::physics::worldToLocal(state, world_point);

    EXPECT_NEAR(round_trip.x, local_point.x, 1e-12);
    EXPECT_NEAR(round_trip.y, local_point.y, 1e-12);
    EXPECT_NEAR(round_trip.z, local_point.z, 1e-12);
}
