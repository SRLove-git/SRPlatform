#include "core/version.hpp"

#include <iostream>

int main()
{
    if (srp::core::kVersion.major != 0 ||
        srp::core::kVersion.minor != 1 ||
        srp::core::kVersion.patch != 0)
    {
        std::cerr << "version constants do not match expected values\n";
        return 1;
    }

    if (srp::core::versionString() != "0.1.0")
    {
        std::cerr << "version string does not match expected value\n";
        return 1;
    }

    return 0;
}
