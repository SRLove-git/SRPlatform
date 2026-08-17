#include "core/config/app_config.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

#include "core/logging.hpp"

namespace srp::core
{

AppConfig defaultAppConfig()
{
    return {};
}

AppConfig loadAppConfig(const std::string& path)
{
    AppConfig config = defaultAppConfig();

    std::ifstream input(path);
    if (!input)
    {
        logWarn("config file not found, using defaults: " + path);
        return config;
    }

    try
    {
        const nlohmann::json data = nlohmann::json::parse(input);

        if (data.contains("window"))
        {
            const auto& window = data.at("window");
            config.window.width = window.value("width", config.window.width);
            config.window.height = window.value("height", config.window.height);
            config.window.title = window.value("title", config.window.title);
        }

        if (data.contains("simulation"))
        {
            const auto& simulation = data.at("simulation");
            config.simulation.fixed_dt = simulation.value("fixed_dt", config.simulation.fixed_dt);
            config.simulation.max_steps_per_frame =
                simulation.value("max_steps_per_frame", config.simulation.max_steps_per_frame);
        }

        if (data.contains("logging"))
        {
            const auto& logging = data.at("logging");
            config.logging.level = logging.value("level", config.logging.level);
        }
    }
    catch (const std::exception& error)
    {
        logError(std::string("failed to parse config file: ") + error.what());
    }

    return config;
}

}  // namespace srp::core
