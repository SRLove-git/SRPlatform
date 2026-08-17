#include "bridge/drone_entity.hpp"

#include "physics/rigid_body.hpp"

#include <gtest/gtest.h>

namespace
{

constexpr double kTimeStep = 1.0 / 60.0;

srp::bridge::DroneParameters testDroneParameters()
{
    return {};
}

}  // namespace

TEST(BridgeDroneEntity, FullThrottleLiftsOff)
{
    srp::bridge::DroneEntity drone(testDroneParameters());
    drone.setThrottle(1.0);

    for (int step = 0; step < 240; ++step)
    {
        drone.step(kTimeStep);
    }

    EXPECT_GT(drone.altitude(), 1.0);
    const auto* body = drone.body();
    ASSERT_NE(body, nullptr);
    EXPECT_GT(body->position.y, 0.5);
}

TEST(BridgeDroneEntity, ZeroThrottleStaysOnGround)
{
    srp::bridge::DroneEntity drone(testDroneParameters());
    drone.setThrottle(0.0);

    for (int step = 0; step < 240; ++step)
    {
        drone.step(kTimeStep);
    }

    EXPECT_LT(drone.altitude(), 0.2);
    EXPECT_NEAR(drone.verticalVelocity(), 0.0, 0.05);
}

TEST(BridgeDroneEntity, ThrottleIsClampedToUnitRange)
{
    srp::bridge::DroneEntity drone(testDroneParameters());

    drone.setThrottle(1.5);
    EXPECT_DOUBLE_EQ(drone.throttle(), 1.0);

    drone.setThrottle(-0.5);
    EXPECT_DOUBLE_EQ(drone.throttle(), 0.0);
}

TEST(BridgeDroneEntity, SensorsFollowBodyState)
{
    srp::bridge::DroneEntity drone(testDroneParameters());
    drone.setThrottle(1.0);

    for (int step = 0; step < 60; ++step)
    {
        drone.step(kTimeStep);
    }

    EXPECT_GT(drone.altitude(), 1.0);
    EXPECT_GT(drone.verticalVelocity(), 0.0);
    EXPECT_NEAR(drone.imu().pitch(), 0.0, 1e-12);
    EXPECT_NEAR(drone.imu().roll(), 0.0, 1e-12);
    EXPECT_NEAR(drone.distanceSensor().beamDirectionWorld().y, -1.0, 1e-12);
    EXPECT_GT(drone.quadcopter().totalThrust(), 0.0);
}

TEST(BridgeDroneEntity, RejectsInvalidParameters)
{
    srp::bridge::DroneParameters parameters = testDroneParameters();
    parameters.chassis_mass = 0.0;
    EXPECT_THROW(
        srp::bridge::DroneEntity{parameters},
        std::invalid_argument);

    parameters = testDroneParameters();
    parameters.max_rotor_angular_velocity_rad_s = 0.0;
    EXPECT_THROW(
        srp::bridge::DroneEntity{parameters},
        std::invalid_argument);
}
