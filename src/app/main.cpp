#include "core/version.hpp"

#include <iostream>

int main()
{
    std::cout << "SRPlatform " << srp::core::versionString() << '\n';
    return 0;
}
