#include "control/pid_controller.hpp"

#include <gtest/gtest.h>

namespace
{

srp::control::PidLimits wideLimits()
{
    srp::control::PidLimits limits;
    limits.output_min = -1000.0;
    limits.output_max = 1000.0;
    limits.integral_min = -1000.0;
    limits.integral_max = 1000.0;
    return limits;
}

}  // namespace

TEST(ControlPidController, ProportionalTermOnly)
{
    srp::control::PidGains gains;
    gains.kp = 2.0;
    srp::control::PidController pid(gains, wideLimits());

    EXPECT_NEAR(pid.compute(3.0, 0.01), 6.0, 1e-12);
}

TEST(ControlPidController, IntegralAccumulatesOverTime)
{
    srp::control::PidGains gains;
    gains.kp = 0.0;
    gains.ki = 1.0;
    srp::control::PidController pid(gains, wideLimits());

    pid.compute(2.0, 0.1);
    EXPECT_NEAR(pid.integral(), 0.2, 1e-12);

    pid.compute(2.0, 0.1);
    EXPECT_NEAR(pid.integral(), 0.4, 1e-12);
    EXPECT_NEAR(pid.output(), 0.4, 1e-12);
}

TEST(ControlPidController, DerivativeUsesErrorChange)
{
    srp::control::PidGains gains;
    gains.kp = 0.0;
    gains.kd = 1.0;
    srp::control::PidController pid(gains, wideLimits());

    EXPECT_DOUBLE_EQ(pid.compute(1.0, 0.1), 0.0);
    EXPECT_NEAR(pid.compute(3.0, 0.1), 20.0, 1e-12);
}

TEST(ControlPidController, OutputIsClamped)
{
    srp::control::PidGains gains;
    gains.kp = 1.0;
    srp::control::PidLimits limits;
    limits.output_min = -1.0;
    limits.output_max = 1.0;
    srp::control::PidController pid(gains, limits);

    EXPECT_DOUBLE_EQ(pid.compute(5.0, 0.01), 1.0);
    EXPECT_DOUBLE_EQ(pid.compute(-5.0, 0.01), -1.0);
}

TEST(ControlPidController, IntegralIsAntiWindupClamped)
{
    srp::control::PidGains gains;
    gains.ki = 1.0;
    srp::control::PidLimits limits;
    limits.integral_min = -1.0;
    limits.integral_max = 1.0;
    srp::control::PidController pid(gains, limits);

    for (int step = 0; step < 10; ++step)
    {
        pid.compute(2.0, 1.0);
    }

    EXPECT_DOUBLE_EQ(pid.integral(), 1.0);
}

TEST(ControlPidController, NonPositiveTimeStepKeepsState)
{
    srp::control::PidGains gains;
    gains.kp = 1.0;
    srp::control::PidController pid(gains, wideLimits());

    pid.compute(2.0, 0.01);
    const double output_before = pid.output();

    EXPECT_DOUBLE_EQ(pid.compute(5.0, 0.0), output_before);
    EXPECT_DOUBLE_EQ(pid.compute(5.0, -0.1), output_before);
}

TEST(ControlPidController, ResetRestoresInitialState)
{
    srp::control::PidGains gains;
    gains.kp = 1.0;
    gains.kd = 1.0;
    srp::control::PidController pid(gains, wideLimits());

    pid.compute(2.0, 0.01);
    pid.compute(4.0, 0.01);
    pid.reset();

    EXPECT_DOUBLE_EQ(pid.output(), 0.0);
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
    // After reset the first derivative term is zero again.
    EXPECT_DOUBLE_EQ(pid.compute(3.0, 0.1), 3.0);
}

TEST(ControlPidController, RejectsInvalidLimits)
{
    srp::control::PidLimits limits;
    limits.output_min = 1.0;
    limits.output_max = 0.0;
    srp::control::PidGains gains;
    EXPECT_THROW(
        srp::control::PidController(gains, limits),
        std::invalid_argument);

    limits = wideLimits();
    limits.integral_min = 2.0;
    limits.integral_max = 1.0;
    EXPECT_THROW(
        srp::control::PidController(gains, limits),
        std::invalid_argument);

    srp::control::PidController pid(gains, wideLimits());
    srp::control::PidLimits bad;
    bad.output_min = 5.0;
    bad.output_max = 4.0;
    EXPECT_THROW(pid.setLimits(bad), std::invalid_argument);
}
