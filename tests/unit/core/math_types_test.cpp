#include "core/math/si_units.hpp"
#include "core/math/types.hpp"

#include <type_traits>

#include <gtest/gtest.h>

TEST(CoreMathTypes, UsesDoublePrecision)
{
    static_assert(std::is_same_v<srp::math::Vec3, glm::dvec3>);
    static_assert(std::is_same_v<srp::math::Mat4, glm::dmat4>);
    static_assert(std::is_same_v<srp::math::Quat, glm::dquat>);
    static_assert(std::is_same_v<srp::math::Scalar, double>);
}

TEST(CoreMathTypes, ConvertsBetweenRadiansAndDegrees)
{
    EXPECT_DOUBLE_EQ(srp::math::radians(180.0), srp::math::kPi);
    EXPECT_DOUBLE_EQ(srp::math::degrees(srp::math::kPi), 180.0);
}

TEST(CoreSiUnits, UsesDoublePrecision)
{
    static_assert(std::is_same_v<srp::math::Length, double>);
    static_assert(std::is_same_v<srp::math::Mass, double>);
    static_assert(std::is_same_v<srp::math::Force, double>);
}
