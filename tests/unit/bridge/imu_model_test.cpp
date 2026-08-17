#include "bridge/imu_model.hpp"

#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

namespace
{

constexpr double kGravity = 9.81;

srp::physics::RigidBodyState levelBody()
{
    srp::physics::RigidBodyState state;
    state.type = srp::physics::RigidBodyType::kDynamic;
    state.mass = 1.0;
    state.orientation = srp::math::Quat(1.0, 0.0, 0.0, 0.0);
    return state;
}

}  // namespace

TEST(BridgeImuModel, LevelBodyAtRestReportsZeroAttitudeAndOneG)
{
    srp::bridge::ImuModel imu;

    imu.update(levelBody(), srp::math::Vec3(0.0));

    EXPECT_NEAR(imu.roll(), 0.0, 1e-12);
    EXPECT_NEAR(imu.pitch(), 0.0, 1e-12);
    EXPECT_NEAR(imu.yaw(), 0.0, 1e-12);
    EXPECT_NEAR(imu.angularVelocity().x, 0.0, 1e-12);
    EXPECT_NEAR(imu.angularVelocity().y, 0.0, 1e-12);
    EXPECT_NEAR(imu.angularVelocity().z, 0.0, 1e-12);
    EXPECT_NEAR(imu.acceleration().x, 0.0, 1e-12);
    EXPECT_NEAR(imu.acceleration().y, kGravity, 1e-12);
    EXPECT_NEAR(imu.acceleration().z, 0.0, 1e-12);
}

TEST(BridgeImuModel, YawIsRotationAboutVerticalAxis)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState state = levelBody();
    state.orientation = glm::angleAxis(0.6, srp::math::Vec3(0.0, 1.0, 0.0));

    imu.update(state, srp::math::Vec3(0.0));

    EXPECT_NEAR(imu.roll(), 0.0, 1e-12);
    EXPECT_NEAR(imu.pitch(), 0.0, 1e-12);
    EXPECT_NEAR(imu.yaw(), 0.6, 1e-12);
}

TEST(BridgeImuModel, PitchIsRotationAboutLateralAxis)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState state = levelBody();
    state.orientation = glm::angleAxis(0.4, srp::math::Vec3(0.0, 0.0, 1.0));

    imu.update(state, srp::math::Vec3(0.0));

    EXPECT_NEAR(imu.roll(), 0.0, 1e-12);
    EXPECT_NEAR(imu.pitch(), 0.4, 1e-12);
    EXPECT_NEAR(imu.yaw(), 0.0, 1e-12);
}

TEST(BridgeImuModel, RollIsRotationAboutForwardAxis)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState state = levelBody();
    state.orientation = glm::angleAxis(-0.5, srp::math::Vec3(1.0, 0.0, 0.0));

    imu.update(state, srp::math::Vec3(0.0));

    EXPECT_NEAR(imu.roll(), -0.5, 1e-12);
    EXPECT_NEAR(imu.pitch(), 0.0, 1e-12);
    EXPECT_NEAR(imu.yaw(), 0.0, 1e-12);
}

TEST(BridgeImuModel, CombinedAttitudeMatchesYawPitchRollOrder)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState state = levelBody();
    state.orientation =
        glm::angleAxis(0.3, srp::math::Vec3(1.0, 0.0, 0.0)) *
        glm::angleAxis(0.2, srp::math::Vec3(0.0, 0.0, 1.0)) *
        glm::angleAxis(0.1, srp::math::Vec3(0.0, 1.0, 0.0));

    imu.update(state, srp::math::Vec3(0.0));

    EXPECT_NEAR(imu.roll(), 0.3, 1e-12);
    EXPECT_NEAR(imu.pitch(), 0.2, 1e-12);
    EXPECT_NEAR(imu.yaw(), 0.1, 1e-12);
}

TEST(BridgeImuModel, GyroReadsBodyFrameAngularVelocity)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState state = levelBody();
    state.orientation = glm::angleAxis(
        srp::math::kPi / 2.0, srp::math::Vec3(0.0, 1.0, 0.0));
    state.angular_velocity = srp::math::Vec3(0.0, 0.0, 3.0);

    imu.update(state, srp::math::Vec3(0.0));

    // A spin about world Z appears as -X in the yawed body frame.
    EXPECT_NEAR(imu.angularVelocity().x, -3.0, 1e-12);
    EXPECT_NEAR(imu.angularVelocity().y, 0.0, 1e-12);
    EXPECT_NEAR(imu.angularVelocity().z, 0.0, 1e-12);
}

TEST(BridgeImuModel, AccelerometerReportsSpecificForceInBodyFrame)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState state = levelBody();
    state.orientation = glm::angleAxis(
        srp::math::kPi / 2.0, srp::math::Vec3(0.0, 0.0, 1.0));

    imu.update(state, srp::math::Vec3(5.0, 0.0, 0.0));

    // Specific force (5, 9.81, 0) in world, pitched 90 deg about Z:
    // x and y swap, with y negated.
    EXPECT_NEAR(imu.acceleration().x, kGravity, 1e-12);
    EXPECT_NEAR(imu.acceleration().y, -5.0, 1e-12);
    EXPECT_NEAR(imu.acceleration().z, 0.0, 1e-12);
}

TEST(BridgeImuModel, BiasShiftsReadings)
{
    srp::bridge::ImuParameters parameters;
    parameters.gyro_bias_rad_s = srp::math::Vec3(0.1, -0.2, 0.3);
    parameters.accelerometer_bias_m_s2 = srp::math::Vec3(1.0, 2.0, 3.0);
    srp::bridge::ImuModel imu(parameters);
    srp::physics::RigidBodyState state = levelBody();
    state.angular_velocity = srp::math::Vec3(0.5, 0.0, 0.0);

    imu.update(state, srp::math::Vec3(0.0));

    EXPECT_NEAR(imu.angularVelocity().x, 0.6, 1e-12);
    EXPECT_NEAR(imu.angularVelocity().y, -0.2, 1e-12);
    EXPECT_NEAR(imu.angularVelocity().z, 0.3, 1e-12);
    EXPECT_NEAR(imu.acceleration().x, 1.0, 1e-12);
    EXPECT_NEAR(imu.acceleration().y, kGravity + 2.0, 1e-12);
    EXPECT_NEAR(imu.acceleration().z, 3.0, 1e-12);
}

TEST(BridgeImuModel, UpdateReplacesPreviousReadings)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState first = levelBody();
    first.orientation = glm::angleAxis(0.5, srp::math::Vec3(0.0, 1.0, 0.0));
    imu.update(first, srp::math::Vec3(0.0));

    imu.update(levelBody(), srp::math::Vec3(0.0));

    EXPECT_NEAR(imu.yaw(), 0.0, 1e-12);
}

TEST(BridgeImuModel, ResetRestoresInitialState)
{
    srp::bridge::ImuModel imu;
    srp::physics::RigidBodyState state = levelBody();
    state.orientation = glm::angleAxis(0.5, srp::math::Vec3(0.0, 1.0, 0.0));
    state.angular_velocity = srp::math::Vec3(1.0, 2.0, 3.0);
    imu.update(state, srp::math::Vec3(4.0, 5.0, 6.0));

    imu.reset();

    EXPECT_EQ(imu.attitude(), srp::math::Vec3(0.0));
    EXPECT_EQ(imu.angularVelocity(), srp::math::Vec3(0.0));
    EXPECT_EQ(imu.acceleration(), srp::math::Vec3(0.0));
    EXPECT_EQ(imu.orientation(), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
}
