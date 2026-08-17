#include "bridge/encoder_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

}  // namespace

TEST(BridgeEncoderModel, ZeroShaftStateProducesZeroReadings)
{
    srp::bridge::EncoderModel encoder;

    encoder.update(0.0, 0.0);

    EXPECT_DOUBLE_EQ(encoder.angleRad(), 0.0);
    EXPECT_DOUBLE_EQ(encoder.angularVelocityRadS(), 0.0);
    EXPECT_EQ(encoder.counts(), 0);
    EXPECT_DOUBLE_EQ(encoder.countsPerSecond(), 0.0);
}

TEST(BridgeEncoderModel, FullRevolutionProducesExpectedCounts)
{
    srp::bridge::EncoderModel encoder;

    encoder.update(kTwoPi, 0.0);

    EXPECT_EQ(encoder.counts(), 100);
}

TEST(BridgeEncoderModel, FractionalAngleTruncatesToFloorCounts)
{
    srp::bridge::EncoderModel encoder;

    encoder.update(kTwoPi * 0.5 + 0.1, 0.0);

    EXPECT_EQ(encoder.counts(), 51);
}

TEST(BridgeEncoderModel, NegativeAngleProducesNegativeCounts)
{
    srp::bridge::EncoderModel encoder;

    encoder.update(-kTwoPi * 0.5, 0.0);

    EXPECT_EQ(encoder.counts(), -50);
}

TEST(BridgeEncoderModel, SpeedPassesThroughAsRadiansPerSecond)
{
    srp::bridge::EncoderModel encoder;

    encoder.update(0.0, 10.0);

    EXPECT_DOUBLE_EQ(encoder.angularVelocityRadS(), 10.0);
    EXPECT_NEAR(encoder.countsPerSecond(), 10.0 * 100.0 / kTwoPi, 1e-12);
}

TEST(BridgeEncoderModel, ResolutionChangesCountScale)
{
    srp::bridge::EncoderParameters parameters;
    parameters.counts_per_revolution = 12;
    srp::bridge::EncoderModel encoder(parameters);

    encoder.update(kTwoPi, 6.0);

    EXPECT_EQ(encoder.counts(), 12);
    EXPECT_NEAR(encoder.countsPerSecond(), 6.0 * 12.0 / kTwoPi, 1e-12);
}

TEST(BridgeEncoderModel, ResetRestoresZero)
{
    srp::bridge::EncoderModel encoder;

    encoder.update(3.0, 5.0);
    encoder.reset();

    EXPECT_DOUBLE_EQ(encoder.angleRad(), 0.0);
    EXPECT_DOUBLE_EQ(encoder.angularVelocityRadS(), 0.0);
    EXPECT_EQ(encoder.counts(), 0);
}

TEST(BridgeEncoderModel, RejectsZeroResolution)
{
    srp::bridge::EncoderParameters parameters;
    parameters.counts_per_revolution = 0;

    EXPECT_THROW(
        srp::bridge::EncoderModel{parameters},
        std::invalid_argument);
}
