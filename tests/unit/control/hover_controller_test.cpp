#include "control/hover_controller.hpp"

#include <gtest/gtest.h>

namespace
{

struct SimpleHoverPlant
{
    double mass{1.0};
    double max_thrust{2.0 * 9.81};
    double gravity{9.81};
    double altitude{0.0};
    double vertical_velocity{0.0};

    void step(double throttle, double dt)
    {
        const double acceleration =
            throttle * max_thrust / mass - gravity;
        vertical_velocity += acceleration * dt;
        altitude += vertical_velocity * dt;
    }
};

}  // namespace

TEST(ControlHoverController, BelowTargetIncreasesThrottle)
{
    srp::control::HoverController controller;

    const double throttle = controller.update(0.5, 0.0, 1.0, 0.01);

    EXPECT_GT(throttle, 0.5);
    EXPECT_GT(controller.verticalVelocityCommand(), 0.0);
}

TEST(ControlHoverController, AboveTargetDecreasesThrottle)
{
    srp::control::HoverController controller;

    const double throttle = controller.update(1.5, 0.0, 1.0, 0.01);

    EXPECT_LT(throttle, 0.5);
    EXPECT_LT(controller.verticalVelocityCommand(), 0.0);
}

TEST(ControlHoverController, AtTargetWithNoVelocityUsesGravityCompensation)
{
    srp::control::HoverController controller;

    const double throttle = controller.update(1.0, 0.0, 1.0, 0.01);

    EXPECT_NEAR(throttle, 0.5, 1e-12);
}

TEST(ControlHoverController, HoldsAltitudeInSimulation)
{
    srp::control::HoverController controller;
    SimpleHoverPlant plant;
    constexpr double kTarget = 1.0;
    constexpr double kDt = 0.01;

    for (int step = 0; step < 2000; ++step)
    {
        const double throttle = controller.update(
            plant.altitude,
            plant.vertical_velocity,
            kTarget,
            kDt);
        plant.step(throttle, kDt);
    }

    EXPECT_NEAR(plant.altitude, kTarget, 0.05);
    EXPECT_NEAR(plant.vertical_velocity, 0.0, 0.1);
    EXPECT_NEAR(controller.throttle(), 0.5, 0.05);
}

TEST(ControlHoverController, ThrottleIsClamped)
{
    srp::control::HoverController controller;

    const double throttle = controller.update(0.0, 0.0, 10.0, 0.01);

    EXPECT_DOUBLE_EQ(throttle, 1.0);
}

TEST(ControlHoverController, ResetRestoresZeroThrottle)
{
    srp::control::HoverController controller;

    controller.update(0.0, 0.0, 10.0, 0.01);
    controller.reset();

    EXPECT_DOUBLE_EQ(controller.throttle(), 0.0);
    EXPECT_DOUBLE_EQ(controller.verticalVelocityCommand(), 0.0);
}

TEST(ControlHoverController, RejectsInvalidLimits)
{
    srp::control::HoverLimits limits;
    srp::control::HoverGains gains;
    limits.max_vertical_velocity_m_s = 0.0;
    EXPECT_THROW(
        srp::control::HoverController(gains, limits),
        std::invalid_argument);

    limits = srp::control::HoverLimits{};
    limits.throttle_min = 1.0;
    limits.throttle_max = 0.0;
    EXPECT_THROW(
        srp::control::HoverController(gains, limits),
        std::invalid_argument);
}
