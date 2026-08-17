#include "physics/collision_shape.hpp"
#include "physics/narrowphase.hpp"

#include <gtest/gtest.h>

namespace
{

constexpr srp::math::Quat kIdentity{1.0, 0.0, 0.0, 0.0};

}  // namespace

TEST(PhysicsNarrowphase, SphereSphereCollides)
{
    srp::physics::SphereShape sphere;
    sphere.radius = 1.0;

    srp::physics::CollisionResult result;
    const bool hit = srp::physics::collide(
        sphere,
        srp::math::Vec3(0.0),
        kIdentity,
        sphere,
        srp::math::Vec3(1.0, 0.0, 0.0),
        kIdentity,
        result);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(result.penetration, 1.0, 1e-12);
    EXPECT_NEAR(result.normal.x, 1.0, 1e-12);
    EXPECT_NEAR(result.point.x, 0.5, 1e-12);
}

TEST(PhysicsNarrowphase, SpherePlaneCollides)
{
    srp::physics::SphereShape sphere;
    sphere.radius = 0.5;

    srp::physics::PlaneShape plane;

    srp::physics::CollisionResult result;
    const bool hit = srp::physics::collide(
        sphere,
        srp::math::Vec3(0.0, 0.25, 0.0),
        kIdentity,
        plane,
        srp::math::Vec3(0.0),
        kIdentity,
        result);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(result.penetration, 0.25, 1e-12);
    EXPECT_NEAR(result.normal.y, -1.0, 1e-12);
    EXPECT_NEAR(result.point.y, 0.0, 1e-12);
}

TEST(PhysicsNarrowphase, SphereBoxCollides)
{
    srp::physics::SphereShape sphere;
    sphere.radius = 0.5;

    srp::physics::BoxShape box;
    box.half_extents = srp::math::Vec3(0.5);

    srp::physics::CollisionResult result;
    const bool hit = srp::physics::collide(
        sphere,
        srp::math::Vec3(0.75, 0.0, 0.0),
        kIdentity,
        box,
        srp::math::Vec3(0.0),
        kIdentity,
        result);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(result.penetration, 0.25, 1e-12);
    EXPECT_NEAR(result.normal.x, 1.0, 1e-12);
    EXPECT_NEAR(result.point.x, 0.5, 1e-12);
}

TEST(PhysicsNarrowphase, BoxPlaneCollides)
{
    srp::physics::BoxShape box;
    box.half_extents = srp::math::Vec3(0.5);

    srp::physics::PlaneShape plane;

    srp::physics::CollisionResult result;
    const bool hit = srp::physics::collide(
        box,
        srp::math::Vec3(0.0, 0.25, 0.0),
        kIdentity,
        plane,
        srp::math::Vec3(0.0),
        kIdentity,
        result);

    ASSERT_TRUE(hit);
    EXPECT_NEAR(result.penetration, 0.25, 1e-12);
    EXPECT_NEAR(result.normal.y, -1.0, 1e-12);
}

TEST(PhysicsNarrowphase, BoxBoxCollides)
{
    srp::physics::BoxShape box;
    box.half_extents = srp::math::Vec3(0.5);

    srp::physics::CollisionResult result;
    const bool hit = srp::physics::collide(
        box,
        srp::math::Vec3(0.0),
        kIdentity,
        box,
        srp::math::Vec3(0.5, 0.0, 0.0),
        kIdentity,
        result);

    ASSERT_TRUE(hit);
    EXPECT_GT(result.penetration, 0.0);
}
