#include "scripting/hot_reload_script_host.hpp"

#include "bridge/actuator_bus.hpp"
#include "bridge/bridge.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>

namespace
{

class RecordingActuatorBus final : public srp::bridge::IActuatorBus
{
public:
    double last_motor{0.0};

    void setMotor(srp::bridge::MotorId id, srp::bridge::ActuatorValue value) override
    {
        (void)id;
        last_motor = value;
    }

    void setServo(srp::bridge::ServoId id, srp::bridge::ActuatorValue angle) override
    {
        (void)id;
        (void)angle;
    }
};

std::filesystem::path writeScript(
    const std::string& name,
    const std::string& content)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_hot_reload_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path path = directory / name;
    std::ofstream(path) << content;
    return path;
}

}  // namespace

TEST(HotReloadScriptHost, LoadsAndRunsFileScript)
{
    const std::filesystem::path path = writeScript(
        "basic.lua",
        "value = 1\n"
        "function update(dt)\n"
        "    value = value + 1\n"
        "end\n");

    srp::scripting::HotReloadScriptHost host;
    EXPECT_TRUE(host.loadFromFile("basic", path));
    EXPECT_TRUE(host.hasScript("basic"));
    EXPECT_TRUE(host.runOnce(0.01));

    const auto value = host.host().getNumber("basic", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 2.0);

    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_hot_reload_test");
}

TEST(HotReloadScriptHost, ReloadsChangedFile)
{
    const std::filesystem::path path = writeScript(
        "reload.lua",
        "value = 1\n"
        "function update(dt) end\n");

    srp::scripting::HotReloadScriptHost host;
    ASSERT_TRUE(host.loadFromFile("reload", path));
    ASSERT_TRUE(host.host().getNumber("reload", "value").has_value());

    std::ofstream(path) <<
        "value = 2\n"
        "function update(dt) end\n";
    const std::filesystem::file_time_type new_time =
        std::filesystem::last_write_time(path) + std::chrono::seconds(5);
    std::filesystem::last_write_time(path, new_time);

    EXPECT_TRUE(host.pollReloads());
    EXPECT_FALSE(host.pollReloads());

    const auto value = host.host().getNumber("reload", "value");
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 2.0);

    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_hot_reload_test");
}

TEST(HotReloadScriptHost, ReportsMissingAndInvalidFiles)
{
    srp::scripting::HotReloadScriptHost host;

    EXPECT_FALSE(host.loadFromFile(
        "missing",
        std::filesystem::temp_directory_path() /
            "srp_hot_reload_does_not_exist" / "nope.lua"));
    EXPECT_TRUE(host.lastError().has_value());

    const std::filesystem::path bad = writeScript(
        "bad.lua",
        "this is not lua (");
    EXPECT_FALSE(host.loadFromFile("bad", bad));
    EXPECT_TRUE(host.lastError().has_value());

    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_hot_reload_test");
}

TEST(HotReloadScriptHost, BindControlEnablesScriptApi)
{
    const std::filesystem::path path = writeScript(
        "control.lua",
        "function update(dt)\n"
        "    set_motor(1, 0.5)\n"
        "end\n");

    srp::scripting::HotReloadScriptHost host;
    EXPECT_TRUE(host.loadFromFile("control", path));

    auto bridge = std::make_shared<srp::bridge::Bridge>();
    auto actuator_bus = std::make_shared<RecordingActuatorBus>();
    bridge->attachActuatorBus(actuator_bus);
    host.bindControl(bridge);
    EXPECT_TRUE(host.loadFromFile("control", path));
    EXPECT_TRUE(host.runOnce(0.01));
    EXPECT_DOUBLE_EQ(actuator_bus->last_motor, 0.5);

    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_hot_reload_test");
}
