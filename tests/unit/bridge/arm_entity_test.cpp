#include "bridge/arm_entity.hpp"

#include <gtest/gtest.h>

namespace srp::bridge
{
namespace
{

TEST(ArmEntityTest, DefaultArmHasThreeJoints)
{
    ArmEntity arm;
    EXPECT_EQ(arm.jointCount(), 3u);
    EXPECT_EQ(arm.kind(), std::string("arm"));
}

TEST(ArmEntityTest, ServoMapsToJointLimit)
{
    ArmEntity arm;
    arm.setServo(0, 1.0);
    EXPECT_NEAR(arm.jointTarget(0), 2.5, 1e-9);
    arm.setServo(0, -0.5);
    EXPECT_NEAR(arm.jointTarget(0), -1.25, 1e-9);
}

TEST(ArmEntityTest, JointApproachesTargetAtLimitedSpeed)
{
    ArmEntity arm;
    arm.setJointTarget(0, 3.0);  // clamped to +2.5 rad

    const double dt = 1.0 / 60.0;
    const int steps = 120;
    for (int i = 0; i < steps; ++i)
    {
        arm.step(dt);
    }

    EXPECT_NEAR(arm.jointAngle(0), 2.5, 0.05);
    EXPECT_NEAR(arm.jointTarget(0), 2.5, 1e-9);
}

TEST(ArmEntityTest, ForwardKinematicsExtendsArm)
{
    ArmEntity arm;
    const math::Vec3 start = arm.basePosition();
    const math::Vec3 end = arm.endEffectorPosition();

    // With all joints at zero, the arm lies along +X.
    EXPECT_GT(end.x, start.x);
    EXPECT_NEAR(end.y, start.y, 1e-9);
    EXPECT_NEAR(end.z, start.z, 1e-9);

    const double total_length =
        arm.jointCount() > 0 ? 0.6 + 0.5 + 0.4 : 0.0;
    EXPECT_NEAR(glm::distance(start, end), total_length, 1e-9);
}

TEST(ArmEntityTest, RaisingShoulderLiftsEndEffector)
{
    ArmEntity arm;
    arm.setJointTarget(0, 1.0);
    for (int i = 0; i < 300; ++i)
    {
        arm.step(1.0 / 60.0);
    }

    const math::Vec3 end = arm.endEffectorPosition();
    EXPECT_GT(end.y, arm.basePosition().y);
}

}  // namespace
}  // namespace srp::bridge
