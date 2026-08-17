#include "scripting/arm_closed_loop_demo.hpp"

#include <gtest/gtest.h>

namespace srp::scripting
{
namespace
{

TEST(ArmDemoTest, ScriptDrivesJoints)
{
    ArmClosedLoopDemo demo;
    const std::string script =
        "function update(dt)\n"
        "    set_servo(1, 1.0)\n"
        "    set_servo(2, -0.5)\n"
        "end\n";

    ASSERT_TRUE(demo.loadScript("controller", script));
    for (int i = 0; i < 120; ++i)
    {
        demo.step(1.0 / 60.0);
    }

    EXPECT_NEAR(demo.arm().jointTarget(0), 2.5, 1e-6);
    EXPECT_NEAR(demo.arm().jointTarget(1), -1.25, 1e-6);
    EXPECT_NEAR(demo.arm().jointAngle(0), 2.5, 0.05);
}

TEST(ArmDemoTest, SensorsExposeJointAngles)
{
    ArmClosedLoopDemo demo;
    const std::string script =
        "function update(dt)\n"
        "    set_servo(1, 1.0)\n"
        "end\n";

    ASSERT_TRUE(demo.loadScript("controller", script));
    for (int i = 0; i < 60; ++i)
    {
        demo.step(1.0 / 60.0);
    }

    EXPECT_GT(demo.arm().jointAngle(0), 0.5);
}

}  // namespace
}  // namespace srp::scripting
