#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace srp::math
{

using Scalar = double;

using Vec2 = glm::dvec2;
using Vec3 = glm::dvec3;
using Vec4 = glm::dvec4;

using Mat3 = glm::dmat3;
using Mat4 = glm::dmat4;

using Quat = glm::dquat;

constexpr Scalar kPi = 3.141592653589793238462643383279502884;
constexpr Scalar kRadiansPerDegree = kPi / 180.0;
constexpr Scalar kDegreesPerRadian = 180.0 / kPi;

inline Scalar radians(Scalar degrees)
{
    return degrees * kRadiansPerDegree;
}

inline Scalar degrees(Scalar radians)
{
    return radians * kDegreesPerRadian;
}

}  // namespace srp::math
