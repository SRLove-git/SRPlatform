#include "core/version.hpp"

#include <string>

namespace srp::core
{

std::string versionString()
{
    return std::to_string(kVersion.major) + "." +
           std::to_string(kVersion.minor) + "." +
           std::to_string(kVersion.patch);
}

}  // namespace srp::core
