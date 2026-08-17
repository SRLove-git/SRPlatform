#include "physics/collision_shape.hpp"
#include "physics/contact.hpp"

#include <gtest/gtest.h>

namespace
{

constexpr srp::math::Quat kIdentity{1.0, 0.0, 0.0, 0.0};

}  // namespace

TEST(PhysicsContact, GeneratesSphereSphereContact)
{
    srp::physics::SphereShape sphere;
    sphere.radius = 1.0;

    const auto contact = srp::physics::generateContact(
        1,
        sphere,
        srp::math::Vec3(0.0),
        kIdentity,
        2,
        sphere,
        srp::math::Vec3(1.0, 0.0, 0.0),
        kIdentity);

    ASSERT_TRUE(contact.has_value());
    EXPECT_EQ(contact->body_a, 1);
    EXPECT_EQ(contact->body_b, 2);
    EXPECT_NEAR(contact->point.penetration, 1.0, 1e-12);
    EXPECT_NEAR(contact->point.normal.x, 1.0, 1e-12);
    EXPECT_NEAR(contact->point.point.x, 0.5, 1e-12);
}

TEST(PhysicsContact, ReturnsNulloptWhenSeparated)
{
    srp::physics::SphereShape sphere;
    sphere.radius = 1.0;

    const auto contact = srp::physics::generateContact(
        1,
        sphere,
        srp::math::Vec3(0.0),
        kIdentity,
        2,
        sphere,
        srp::math::Vec3(3.0, 0.0, 0.0),
        kIdentity);

    EXPECT_FALSE(contact.has_value());
}
