#include "scripting/drone_closed_loop_demo.hpp"
#include "scripting/lua_script_host.hpp"

#include <gtest/gtest.h>

namespace
{

constexpr double kTimeStep = 1.0 / 60.0;

constexpr const char* kHoverScript =
    "target_altitude = 1.0\n"
    "hover_throttle = 0.58\n"
    "kp = 0.5\n"
    "kd = 0.3\n"
    "function clamp(value, low, high)\n"
    "    if value < low then return low end\n"
    "    if value > high then return high end\n"
    "    return value\n"
    "end\n"
    "function update(dt)\n"
    "    altitude = read_sensor(1)\n"
    "    vertical_velocity = read_sensor(2)\n"
    "    throttle = hover_throttle + kp * (target_altitude - altitude) - kd * vertical_velocity\n"
    "    set_motor(1, clamp(throttle, 0.0, 1.0))\n"
    "end\n";

}  // namespace

TEST(DroneClosedLoopDemo, HoverScriptHoldsAltitude)
{
    srp::scripting::DroneClosedLoopDemo demo;

    EXPECT_TRUE(demo.loadScript("hover", kHoverScript));

    for (int step = 0; step < 600; ++step)
    {
        EXPECT_TRUE(demo.step(kTimeStep));
    }

    EXPECT_NEAR(demo.drone().altitude(), 1.0, 0.15);
    EXPECT_NEAR(demo.drone().verticalVelocity(), 0.0, 0.5);
    EXPECT_GT(demo.drone().throttle(), 0.0);
    EXPECT_LT(demo.drone().throttle(), 1.0);
}

TEST(DroneClosedLoopDemo, ScriptCanReadDroneSensors)
{
    srp::scripting::DroneClosedLoopDemo demo;

    constexpr const char* script =
        "observed_altitude = 0\n"
        "observed_velocity = 0\n"
        "function update(dt)\n"
        "    set_motor(1, 1.0)\n"
        "    observed_altitude = read_sensor(1)\n"
        "    observed_velocity = read_sensor(2)\n"
        "end\n";

    EXPECT_TRUE(demo.loadScript("sensors", script));
    EXPECT_TRUE(demo.step(kTimeStep));

    const auto altitude = demo.host().getNumber("sensors", "observed_altitude");
    const auto velocity = demo.host().getNumber("sensors", "observed_velocity");

    ASSERT_TRUE(altitude.has_value());
    ASSERT_TRUE(velocity.has_value());
    EXPECT_GE(*altitude, 0.0);
    EXPECT_GE(*velocity, 0.0);
}

TEST(DroneClosedLoopDemo, InvalidScriptIsRejected)
{
    srp::scripting::DroneClosedLoopDemo demo;

    EXPECT_FALSE(demo.loadScript("bad", "this is not lua ("));
}
