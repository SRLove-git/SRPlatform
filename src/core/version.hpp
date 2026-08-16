#pragma once

#include <string>

namespace srp::core
{

struct Version
{
    int major;
    int minor;
    int patch;
};

inline constexpr Version kVersion{0, 1, 0};

std::string versionString();

}  // namespace srp::core
