#include "scripting/lua_script_host.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace srp::scripting
{
namespace
{

TEST(SensorOverrideTest, OverrideTakesPriorityOverBridge)
{
    LuaScriptHost host;
    host.setSensorOverride(
        11,
        []()
        {
            return std::optional<double>(0.42);
        });

    const std::string script =
        "function update(dt)\n"
        "    value = read_sensor(11)\n"
        "end\n";
    ASSERT_TRUE(host.load("controller", script));

    // A script that reads the override must not error.
    EXPECT_TRUE(host.runOnce(1.0 / 60.0));
    const std::optional<double> value = host.getNumber("controller", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 0.42);
}

TEST(SensorOverrideTest, ClearOverridesRestoresBridgeBehavior)
{
    LuaScriptHost host;
    host.setSensorOverride(
        11,
        []()
        {
            return std::optional<double>(1.0);
        });
    host.clearSensorOverrides();

    const std::string script =
        "function update(dt)\n"
        "    value = read_sensor(11)\n"
        "end\n";
    ASSERT_TRUE(host.load("controller", script));
    EXPECT_TRUE(host.runOnce(1.0 / 60.0));

    // Without a bridge or override the reading is NaN.
    const std::optional<double> value = host.getNumber("controller", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_TRUE(std::isnan(*value));
}

}  // namespace
}  // namespace srp::scripting
