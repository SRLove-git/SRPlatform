#pragma once

#include <string_view>

namespace srp::core
{

void initLogging(std::string_view level);
void logInfo(std::string_view message);
void logWarn(std::string_view message);
void logError(std::string_view message);

}  // namespace srp::core
