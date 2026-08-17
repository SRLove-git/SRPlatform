#include "physics/collision_shape.hpp"

#include <gtest/gtest.h>

TEST(PhysicsCollisionShape, BoxHasExpectedDefaults)
{
    const srp::physics::BoxShape box;
    EXPECT_EQ(srp::physics::shapeType(box), srp::physics::CollisionShapeType::kBox);
    EXPECT_DOUBLE_EQ(box.half_extents.x, 0.5);
    EXPECT_DOUBLE_EQ(box.half_extents.y, 0.5);
    EXPECT_DOUBLE_EQ(box.half_extents.z, 0.5);
}

TEST(PhysicsCollisionShape, SphereHasExpectedDefaults)
{
    const srp::physics::SphereShape sphere;
    EXPECT_EQ(srp::physics::shapeType(sphere), srp::physics::CollisionShapeType::kSphere);
    EXPECT_DOUBLE_EQ(sphere.radius, 0.5);
}

TEST(PhysicsCollisionShape, PlaneHasExpectedDefaults)
{
    const srp::physics::PlaneShape plane;
    EXPECT_EQ(srp::physics::shapeType(plane), srp::physics::CollisionShapeType::kPlane);
    EXPECT_DOUBLE_EQ(plane.normal.x, 0.0);
    EXPECT_DOUBLE_EQ(plane.normal.y, 1.0);
    EXPECT_DOUBLE_EQ(plane.normal.z, 0.0);
    EXPECT_DOUBLE_EQ(plane.offset, 0.0);
}

TEST(PhysicsCollisionShape, CylinderHasExpectedDefaults)
{
    const srp::physics::CylinderShape cylinder;
    EXPECT_EQ(srp::physics::shapeType(cylinder), srp::physics::CollisionShapeType::kCylinder);
    EXPECT_DOUBLE_EQ(cylinder.half_height, 0.5);
    EXPECT_DOUBLE_EQ(cylinder.radius, 0.5);
}

TEST(PhysicsCollisionShape, ConvexHullStoresPoints)
{
    srp::physics::ConvexHullShape hull;
    hull.points = {
        srp::math::Vec3(-1.0, -1.0, -1.0),
        srp::math::Vec3(1.0, -1.0, -1.0),
        srp::math::Vec3(0.0, 1.0, 0.0)};

    EXPECT_EQ(srp::physics::shapeType(hull), srp::physics::CollisionShapeType::kConvexHull);
    EXPECT_EQ(hull.points.size(), 3);
}
