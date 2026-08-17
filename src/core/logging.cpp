#include "core/logging.hpp"

#include <filesystem>
#include <memory>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

namespace srp::core
{

void initLogging(std::string_view level)
{
    std::filesystem::create_directories("logs");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/srp.log", true);
    auto logger = std::make_shared<spdlog::logger>("srp", std::move(file_sink));
    logger->set_level(spdlog::level::from_str(std::string(level)));

    spdlog::set_default_logger(std::move(logger));
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
}

void logInfo(std::string_view message)
{
    spdlog::info("{}", message);
}

void logWarn(std::string_view message)
{
    spdlog::warn("{}", message);
}

void logError(std::string_view message)
{
    spdlog::error("{}", message);
}

}  // namespace srp::core
