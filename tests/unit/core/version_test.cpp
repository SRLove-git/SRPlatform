#include "core/version.hpp"

#include <gtest/gtest.h>

TEST(CoreVersion, ConstantsMatchExpectedValues)
{
    EXPECT_EQ(srp::core::kVersion.major, 0);
    EXPECT_EQ(srp::core::kVersion.minor, 1);
    EXPECT_EQ(srp::core::kVersion.patch, 0);
}

TEST(CoreVersion, StringMatchesExpectedValue)
{
    EXPECT_EQ(srp::core::versionString(), "0.1.0");
}
