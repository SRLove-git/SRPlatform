#include "bridge/propeller_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

}  // namespace

TEST(BridgePropellerModel, ZeroSpeedProducesNoAerodynamicOutput)
{
    srp::bridge::PropellerModel propeller;

    EXPECT_DOUBLE_EQ(propeller.thrust(), 0.0);
    EXPECT_DOUBLE_EQ(propeller.torque(), 0.0);
    EXPECT_DOUBLE_EQ(propeller.power(), 0.0);
}

TEST(BridgePropellerModel, ThrustAndTorqueScaleQuadraticallyWithSpeed)
{
    srp::bridge::PropellerModel propeller;

    propeller.setAngularVelocity(500.0);
    const double thrust_base = propeller.thrust();
    const double torque_base = propeller.torque();
    const double power_base = propeller.power();

    propeller.setAngularVelocity(1000.0);

    EXPECT_NEAR(propeller.thrust(), 4.0 * thrust_base, 1e-9);
    EXPECT_NEAR(propeller.torque(), 4.0 * torque_base, 1e-9);
    EXPECT_NEAR(propeller.power(), 8.0 * power_base, 1e-9);
}

TEST(BridgePropellerModel, ReverseSpinFlipsThrustAndTorqueSigns)
{
    srp::bridge::PropellerModel propeller;

    propeller.setAngularVelocity(800.0);
    const double thrust_forward = propeller.thrust();
    const double torque_forward = propeller.torque();

    propeller.setAngularVelocity(-800.0);

    EXPECT_NEAR(propeller.thrust(), -thrust_forward, 1e-12);
    EXPECT_NEAR(propeller.torque(), -torque_forward, 1e-12);
    EXPECT_GT(propeller.torque(), 0.0);
}

TEST(BridgePropellerModel, PowerMatchesTorqueMagnitudeTimesSpeed)
{
    srp::bridge::PropellerModel propeller;

    propeller.setAngularVelocity(1200.0);

    EXPECT_NEAR(
        propeller.power(),
        std::abs(propeller.torque() * propeller.angularVelocity()),
        1e-9);
}

TEST(BridgePropellerModel, OutputsMatchDocumentedFormula)
{
    srp::bridge::PropellerModel propeller;
    constexpr double kOmega = 1000.0;

    propeller.setAngularVelocity(kOmega);

    const double n = kOmega / kTwoPi;
    const double diameter = 0.127;
    const double d4 = diameter * diameter * diameter * diameter;
    const double d5 = d4 * diameter;
    const double rho = 1.225;

    EXPECT_NEAR(
        propeller.thrust(),
        0.11 * rho * n * std::abs(n) * d4,
        1e-12);
    EXPECT_NEAR(
        propeller.torque(),
        -0.011 * rho * n * std::abs(n) * d5,
        1e-12);
    EXPECT_NEAR(
        propeller.power(),
        0.011 * rho * std::abs(n * n * n) * kTwoPi * d5,
        1e-12);
}

TEST(BridgePropellerModel, DefaultParametersProducePlausibleMagnitudes)
{
    srp::bridge::PropellerModel propeller;

    propeller.setAngularVelocity(1000.0);

    EXPECT_GT(propeller.thrust(), 0.1);
    EXPECT_LT(propeller.thrust(), 10.0);
    EXPECT_GT(propeller.power(), 0.0);
}

TEST(BridgePropellerModel, ResetRestoresZero)
{
    srp::bridge::PropellerModel propeller;

    propeller.setAngularVelocity(900.0);
    propeller.reset();

    EXPECT_DOUBLE_EQ(propeller.angularVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(propeller.thrust(), 0.0);
    EXPECT_DOUBLE_EQ(propeller.torque(), 0.0);
}

TEST(BridgePropellerModel, RejectsInvalidParameters)
{
    srp::bridge::PropellerParameters parameters;
    parameters.diameter_m = 0.0;
    EXPECT_THROW(srp::bridge::PropellerModel{parameters}, std::invalid_argument);

    parameters = srp::bridge::PropellerParameters{};
    parameters.thrust_coefficient = 0.0;
    EXPECT_THROW(srp::bridge::PropellerModel{parameters}, std::invalid_argument);

    parameters = srp::bridge::PropellerParameters{};
    parameters.torque_coefficient = -0.01;
    EXPECT_THROW(srp::bridge::PropellerModel{parameters}, std::invalid_argument);

    parameters = srp::bridge::PropellerParameters{};
    parameters.air_density_kg_m3 = 0.0;
    EXPECT_THROW(srp::bridge::PropellerModel{parameters}, std::invalid_argument);
}
