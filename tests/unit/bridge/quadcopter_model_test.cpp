#include "bridge/quadcopter_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{

srp::bridge::QuadcopterParameters testQuadcopterParameters()
{
    srp::bridge::QuadcopterParameters parameters;
    parameters.arm_length_m = 0.25;
    return parameters;
}

void spinAll(srp::bridge::QuadcopterForceModel& quad, double angular_velocity)
{
    for (std::size_t i = 0; i < quad.kRotorCount; ++i)
    {
        quad.setRotorAngularVelocity(i, angular_velocity);
    }
}

}  // namespace

TEST(BridgeQuadcopterModel, ZeroSpeedProducesNoForceOrTorque)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());

    EXPECT_DOUBLE_EQ(quad.totalThrust(), 0.0);
    EXPECT_DOUBLE_EQ(quad.totalPower(), 0.0);
    EXPECT_EQ(quad.force(), srp::math::Vec3(0.0));
    EXPECT_EQ(quad.torque(), srp::math::Vec3(0.0));
}

TEST(BridgeQuadcopterModel, EqualSpeedsProducePureLift)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());
    spinAll(quad, 900.0);

    const double single_thrust = quad.rotor(0).thrust();

    EXPECT_NEAR(quad.totalThrust(), 4.0 * single_thrust, 1e-12);
    EXPECT_NEAR(quad.force().y, quad.totalThrust(), 1e-12);
    EXPECT_NEAR(quad.force().x, 0.0, 1e-12);
    EXPECT_NEAR(quad.force().z, 0.0, 1e-12);

    // Adjacent rotors spin in opposite directions, so drag yaw cancels.
    EXPECT_NEAR(quad.torque().x, 0.0, 1e-12);
    EXPECT_NEAR(quad.torque().y, 0.0, 1e-12);
    EXPECT_NEAR(quad.torque().z, 0.0, 1e-12);
}

TEST(BridgeQuadcopterModel, DifferentialSpeedProducesPitchAndRoll)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());
    spinAll(quad, 800.0);

    quad.setRotorAngularVelocity(0, 1200.0);  // faster front rotor
    quad.setRotorAngularVelocity(1, 1200.0);  // faster right rotor

    const double arm = testQuadcopterParameters().arm_length_m;
    const double front_thrust = quad.rotor(0).thrust();
    const double right_thrust = quad.rotor(1).thrust();
    const double back_thrust = quad.rotor(2).thrust();
    const double left_thrust = quad.rotor(3).thrust();

    const double expected_pitch = arm * (std::abs(front_thrust) - std::abs(back_thrust));
    const double expected_roll = arm * (std::abs(right_thrust) - std::abs(left_thrust));

    EXPECT_NEAR(quad.torque().x, expected_roll, 1e-9);
    EXPECT_NEAR(quad.torque().z, expected_pitch, 1e-9);
    EXPECT_GT(quad.torque().x, 0.0);
    EXPECT_GT(quad.torque().z, 0.0);
}

TEST(BridgeQuadcopterModel, DifferentialSpeedProducesYaw)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());
    spinAll(quad, 800.0);

    // Front and back spin +Y, so speeding them up breaks yaw cancellation.
    quad.setRotorAngularVelocity(0, 1200.0);
    quad.setRotorAngularVelocity(2, 1200.0);

    const double expected_yaw =
        quad.rotor(0).torque() +
        quad.rotor(1).torque() +
        quad.rotor(2).torque() +
        quad.rotor(3).torque();

    EXPECT_NEAR(quad.torque().y, expected_yaw, 1e-12);
    EXPECT_NEAR(quad.torque().x, 0.0, 1e-9);
    EXPECT_NEAR(quad.torque().z, 0.0, 1e-9);
}

TEST(BridgeQuadcopterModel, RotorPositionsFollowPlusLayout)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());
    const double arm = testQuadcopterParameters().arm_length_m;

    EXPECT_EQ(quad.rotorPosition(0), srp::math::Vec3(arm, 0.0, 0.0));
    EXPECT_EQ(quad.rotorPosition(1), srp::math::Vec3(0.0, 0.0, -arm));
    EXPECT_EQ(quad.rotorPosition(2), srp::math::Vec3(-arm, 0.0, 0.0));
    EXPECT_EQ(quad.rotorPosition(3), srp::math::Vec3(0.0, 0.0, arm));
}

TEST(BridgeQuadcopterModel, RotorSpinDirectionsAlternate)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());

    quad.setRotorAngularVelocity(0, 1000.0);
    quad.setRotorAngularVelocity(1, 1000.0);

    EXPECT_GT(quad.rotorAngularVelocity(0), 0.0);
    EXPECT_LT(quad.rotorAngularVelocity(1), 0.0);
}

TEST(BridgeQuadcopterModel, TotalPowerSumsRotorPowers)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());
    spinAll(quad, 950.0);

    double expected_power = 0.0;
    for (std::size_t i = 0; i < quad.kRotorCount; ++i)
    {
        expected_power += quad.rotor(i).power();
    }

    EXPECT_NEAR(quad.totalPower(), expected_power, 1e-12);
}

TEST(BridgeQuadcopterModel, ResetRestoresZero)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());
    spinAll(quad, 900.0);

    quad.reset();

    EXPECT_DOUBLE_EQ(quad.totalThrust(), 0.0);
    EXPECT_EQ(quad.torque(), srp::math::Vec3(0.0));
}

TEST(BridgeQuadcopterModel, RejectsInvalidParameters)
{
    srp::bridge::QuadcopterParameters parameters = testQuadcopterParameters();
    parameters.arm_length_m = 0.0;
    EXPECT_THROW(
        srp::bridge::QuadcopterForceModel{parameters},
        std::invalid_argument);
}

TEST(BridgeQuadcopterModel, RejectsOutOfRangeRotorIndex)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());

    EXPECT_THROW(
        quad.setRotorAngularVelocity(4, 1000.0),
        std::out_of_range);
    EXPECT_THROW(quad.rotorAngularVelocity(4), std::out_of_range);
    EXPECT_THROW(quad.rotor(4), std::out_of_range);
    EXPECT_THROW(quad.rotorPosition(4), std::out_of_range);
}

TEST(BridgeQuadcopterModel, HoverThrustIsPlausible)
{
    srp::bridge::QuadcopterForceModel quad(testQuadcopterParameters());

    // ~1 kg craft needs about 9.81 N of total lift.
    double angular_velocity = 0.0;
    for (int iteration = 0; iteration < 200; ++iteration)
    {
        spinAll(quad, angular_velocity);
        if (quad.totalThrust() >= 9.81)
        {
            break;
        }
        angular_velocity += 10.0;
    }

    EXPECT_GE(quad.totalThrust(), 9.81);
    EXPECT_LT(quad.totalThrust(), 9.81 * 1.1);
}
