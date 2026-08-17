#include "core/config/app_config.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

TEST(CoreAppConfig, DefaultsAreCorrect)
{
    const auto defaults = srp::core::defaultAppConfig();
    EXPECT_EQ(defaults.window.width, 1440);
    EXPECT_EQ(defaults.window.height, 900);
    EXPECT_EQ(defaults.window.title, "SRPlatform");
    EXPECT_EQ(defaults.simulation.max_steps_per_frame, 8);
}

TEST(CoreAppConfig, ParsesJsonFile)
{
    const auto temp_path =
        std::filesystem::temp_directory_path() / "srplatform_config_test.json";

    {
        std::ofstream output(temp_path);
        output << R"({
  "window": { "width": 1280, "height": 720, "title": "Test Platform" },
  "simulation": { "fixed_dt": 0.01, "max_steps_per_frame": 12 },
  "logging": { "level": "debug" }
})";
    }

    const auto config = srp::core::loadAppConfig(temp_path.string());
    EXPECT_EQ(config.window.width, 1280);
    EXPECT_EQ(config.window.height, 720);
    EXPECT_EQ(config.window.title, "Test Platform");
    EXPECT_DOUBLE_EQ(config.simulation.fixed_dt, 0.01);
    EXPECT_EQ(config.simulation.max_steps_per_frame, 12);
    EXPECT_EQ(config.logging.level, "debug");

    std::filesystem::remove(temp_path);
}
