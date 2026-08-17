#pragma once

#include "core/math/types.hpp"

#include <variant>
#include <vector>

namespace srp::physics
{

enum class CollisionShapeType
{
    kBox,
    kSphere,
    kPlane,
    kCylinder,
    kConvexHull
};

struct BoxShape
{
    math::Vec3 half_extents{0.5};
};

struct SphereShape
{
    math::Scalar radius{0.5};
};

struct PlaneShape
{
    math::Vec3 normal{0.0, 1.0, 0.0};
    math::Scalar offset{0.0};
};

struct CylinderShape
{
    math::Scalar half_height{0.5};
    math::Scalar radius{0.5};
};

struct ConvexHullShape
{
    std::vector<math::Vec3> points;
};

using CollisionShape =
    std::variant<BoxShape, SphereShape, PlaneShape, CylinderShape, ConvexHullShape>;

CollisionShapeType shapeType(const CollisionShape& shape);

}  // namespace srp::physics
