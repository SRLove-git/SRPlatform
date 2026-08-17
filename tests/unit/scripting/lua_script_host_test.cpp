#include "scripting/lua_script_host.hpp"

#include <gtest/gtest.h>

namespace
{

constexpr const char* kCounterScript =
    "value = 0\n"
    "function update(dt)\n"
    "    value = value + dt\n"
    "end\n";

constexpr const char* kReloadedCounterScript =
    "value = 100\n"
    "function update(dt)\n"
    "    value = value + dt\n"
    "end\n";

}  // namespace

TEST(LuaScriptHost, LoadsAndRunsUpdateFunction)
{
    srp::scripting::LuaScriptHost host;

    EXPECT_TRUE(host.load("counter", kCounterScript));
    EXPECT_TRUE(host.hasScript("counter"));
    EXPECT_EQ(host.scriptCount(), 1U);

    EXPECT_TRUE(host.runOnce(0.25));
    EXPECT_TRUE(host.runOnce(0.25));

    const auto value = host.getNumber("counter", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 0.5, 1e-12);
}

TEST(LuaScriptHost, ReloadRebuildsScriptFromCachedSource)
{
    srp::scripting::LuaScriptHost host;

    EXPECT_TRUE(host.load("counter", kCounterScript));
    EXPECT_TRUE(host.runOnce(1.0));
    EXPECT_TRUE(host.reload("counter"));
    EXPECT_TRUE(host.runOnce(1.0));

    const auto value = host.getNumber("counter", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 1.0, 1e-12);
}

TEST(LuaScriptHost, LoadingSameIdReplacesPreviousScript)
{
    srp::scripting::LuaScriptHost host;

    EXPECT_TRUE(host.load("counter", kCounterScript));
    EXPECT_TRUE(host.runOnce(0.0));
    EXPECT_TRUE(host.load("counter", kReloadedCounterScript));
    EXPECT_TRUE(host.runOnce(0.0));

    const auto value = host.getNumber("counter", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_NEAR(*value, 100.0, 1e-12);
    EXPECT_EQ(host.scriptCount(), 1U);
}

TEST(LuaScriptHost, ScriptsUseIsolatedEnvironments)
{
    srp::scripting::LuaScriptHost host;

    EXPECT_TRUE(host.load("a", "value = 1\n"));
    EXPECT_TRUE(host.load("b", "value = 2\n"));

    const auto value_a = host.getNumber("a", "value");
    const auto value_b = host.getNumber("b", "value");

    ASSERT_TRUE(value_a.has_value());
    ASSERT_TRUE(value_b.has_value());
    EXPECT_DOUBLE_EQ(*value_a, 1.0);
    EXPECT_DOUBLE_EQ(*value_b, 2.0);
}

TEST(LuaScriptHost, RejectsInvalidLuaSource)
{
    srp::scripting::LuaScriptHost host;

    EXPECT_FALSE(host.load("broken", "function update(("));
    EXPECT_FALSE(host.hasScript("broken"));
    EXPECT_EQ(host.scriptCount(), 0U);
    EXPECT_TRUE(host.lastError().has_value());
}

TEST(LuaScriptHost, ReloadMissingScriptFails)
{
    srp::scripting::LuaScriptHost host;

    EXPECT_FALSE(host.reload("missing"));
    EXPECT_TRUE(host.lastError().has_value());
}

TEST(LuaScriptHost, ScriptWithoutUpdateStillLoads)
{
    srp::scripting::LuaScriptHost host;

    EXPECT_TRUE(host.load("no_update", "value = 42\n"));
    EXPECT_TRUE(host.runOnce(0.0));

    const auto value = host.getNumber("no_update", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 42.0);
}
