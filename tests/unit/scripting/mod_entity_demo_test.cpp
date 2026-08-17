#include "scripting/mod_entity_demo.hpp"

#include "bridge/car_entity.hpp"
#include "physics/rigid_body.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace
{

constexpr double kTimeStep = 1.0 / 60.0;

std::filesystem::path findExampleMod()
{
    std::filesystem::path current = std::filesystem::current_path();
    for (int depth = 0; depth < 6; ++depth)
    {
        const std::filesystem::path candidate =
            current / "assets" / "mods" / "rc_car_demo";
        if (std::filesystem::is_directory(candidate))
        {
            return candidate;
        }
        if (current == current.parent_path())
        {
            break;
        }
        current = current.parent_path();
    }
    return {};
}

std::filesystem::path writeTempMod(
    const std::string& script_body,
    bool with_blueprint)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        "srp_mod_entity_demo" / "test_mod";
    std::filesystem::create_directories(directory / "scripts");

    nlohmann::json manifest;
    manifest["id"] = "com.example.temp";
    manifest["name"] = "Temp Mod";
    manifest["version"] = "1.0.0";
    manifest["entry"] = "scripts/main.lua";
    if (with_blueprint)
    {
        manifest["blueprint"] = "blueprint.json";
    }
    std::ofstream(directory / "mod.json") << manifest.dump(2);

    if (with_blueprint)
    {
        nlohmann::json blueprint;
        blueprint["id"] = "temp_car";
        blueprint["kind"] = "car";
        blueprint["parameters"]["battery"]["full_charge_voltage_v"] = 12.0;
        blueprint["parameters"]["battery"]["empty_charge_voltage_v"] = 9.0;
        blueprint["parameters"]["battery"]["internal_resistance_ohm"] = 0.05;
        std::ofstream(directory / "blueprint.json") << blueprint.dump(2);
    }

    const std::string source =
        "elapsed = 0\n"
        "function update(dt)\n"
        "    elapsed = elapsed + dt\n" +
        script_body +
        "end\n";
    std::ofstream(directory / "scripts" / "main.lua") << source;
    return directory;
}

void cleanup()
{
    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_mod_entity_demo");
}

}  // namespace

TEST(ModEntityDemo, LoadsExampleModAndRunsVehicle)
{
    const std::filesystem::path mod_directory = findExampleMod();
    ASSERT_FALSE(mod_directory.empty());

    srp::scripting::ModEntityDemo demo;
    std::string error;
    ASSERT_TRUE(demo.load(mod_directory, error)) << error;

    ASSERT_NE(demo.entity(), nullptr);
    EXPECT_STREQ(demo.entity()->kind(), "car");

    for (int step = 0; step < 120; ++step)
    {
        EXPECT_TRUE(demo.step(kTimeStep));
    }

    auto* car = dynamic_cast<srp::bridge::CarEntity*>(demo.entity());
    ASSERT_NE(car, nullptr);
    const auto* chassis = car->chassisBody();
    ASSERT_NE(chassis, nullptr);
    EXPECT_GT(chassis->position.x, 0.0);
    EXPECT_EQ(car->recorder().size(), 120U);
}

TEST(ModEntityDemo, HotReloadChangesRunningVehicleBehavior)
{
    const std::filesystem::path mod_directory = writeTempMod(
        "    set_motor(1, 1.0)\n"
        "    set_servo(1, 0.0)\n",
        true);

    srp::scripting::ModEntityDemo demo;
    std::string error;
    ASSERT_TRUE(demo.load(mod_directory, error)) << error;

    auto* car = dynamic_cast<srp::bridge::CarEntity*>(demo.entity());
    ASSERT_NE(car, nullptr);

    for (int step = 0; step < 30; ++step)
    {
        EXPECT_TRUE(demo.step(kTimeStep));
    }

    const double forward_x = car->chassisBody()->position.x;
    EXPECT_GT(forward_x, 0.0);

    std::ofstream(mod_directory / "scripts" / "main.lua")
        << "elapsed = 0\n"
           "function update(dt)\n"
           "    elapsed = elapsed + dt\n"
           "    set_motor(1, -1.0)\n"
           "    set_servo(1, 0.0)\n"
           "end\n";
    const std::filesystem::file_time_type new_time =
        std::filesystem::last_write_time(
            mod_directory / "scripts" / "main.lua") +
        std::chrono::seconds(5);
    std::filesystem::last_write_time(
        mod_directory / "scripts" / "main.lua",
        new_time);

    EXPECT_TRUE(demo.pollReloads());

    for (int step = 0; step < 180; ++step)
    {
        EXPECT_TRUE(demo.step(kTimeStep));
    }

    EXPECT_LT(car->chassisBody()->position.x, forward_x);
    cleanup();
}

TEST(ModEntityDemo, ReportsMissingBlueprint)
{
    const std::filesystem::path mod_directory =
        writeTempMod("    set_motor(1, 1.0)\n", false);

    srp::scripting::ModEntityDemo demo;
    std::string error;
    EXPECT_FALSE(demo.load(mod_directory, error));
    EXPECT_FALSE(error.empty());
    cleanup();
}
