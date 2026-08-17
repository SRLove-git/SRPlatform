#pragma once

#include <cstddef>
#include <string>

namespace srp::core
{

struct WindowConfig
{
    int width{800};
    int height{600};
    std::string title{"SRPlatform"};
};

struct SimulationConfig
{
    double fixed_dt{1.0 / 60.0};
    std::size_t max_steps_per_frame{8};
};

struct LoggingConfig
{
    std::string level{"info"};
};

struct AppConfig
{
    WindowConfig window;
    SimulationConfig simulation;
    LoggingConfig logging;
};

AppConfig defaultAppConfig();
AppConfig loadAppConfig(const std::string& path);

}  // namespace srp::core
