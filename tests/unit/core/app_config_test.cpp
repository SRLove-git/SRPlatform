#include "core/config/app_config.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

int main()
{
    const auto defaults = srp::core::defaultAppConfig();
    if (defaults.window.width != 800 ||
        defaults.window.height != 600 ||
        defaults.simulation.max_steps_per_frame != 8)
    {
        std::cerr << "default config values are incorrect\n";
        return 1;
    }

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
    if (config.window.width != 1280 ||
        config.window.height != 720 ||
        config.window.title != "Test Platform" ||
        config.simulation.fixed_dt != 0.01 ||
        config.simulation.max_steps_per_frame != 12 ||
        config.logging.level != "debug")
    {
        std::cerr << "parsed config values are incorrect\n";
        std::filesystem::remove(temp_path);
        return 1;
    }

    std::filesystem::remove(temp_path);
    return 0;
}
