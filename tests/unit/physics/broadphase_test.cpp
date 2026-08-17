#include "physics/aabb.hpp"
#include "physics/broadphase.hpp"
#include "physics/collision_shape.hpp"

#include <set>
#include <utility>

#include <gtest/gtest.h>

namespace
{

std::set<std::pair<srp::physics::BodyId, srp::physics::BodyId>> toSet(
    const std::vector<srp::physics::BodyPair>& pairs)
{
    std::set<std::pair<srp::physics::BodyId, srp::physics::BodyId>> result;
    for (const auto& pair : pairs)
    {
        result.insert({pair.first, pair.second});
    }
    return result;
}

}  // namespace

TEST(PhysicsBroadphase, ComputesBoxAabb)
{
    srp::physics::BoxShape box;
    box.half_extents = srp::math::Vec3(1.0, 2.0, 3.0);

    const auto aabb = srp::physics::computeAabb(
        box,
        srp::math::Vec3(0.0),
        srp::math::Quat(1.0, 0.0, 0.0, 0.0));

    EXPECT_TRUE(aabb.is_finite);
    EXPECT_DOUBLE_EQ(aabb.min.x, -1.0);
    EXPECT_DOUBLE_EQ(aabb.min.y, -2.0);
    EXPECT_DOUBLE_EQ(aabb.min.z, -3.0);
    EXPECT_DOUBLE_EQ(aabb.max.x, 1.0);
    EXPECT_DOUBLE_EQ(aabb.max.y, 2.0);
    EXPECT_DOUBLE_EQ(aabb.max.z, 3.0);
}

TEST(PhysicsBroadphase, PlaneIsUnbounded)
{
    const srp::physics::PlaneShape plane;
    const auto aabb = srp::physics::computeAabb(
        plane,
        srp::math::Vec3(0.0),
        srp::math::Quat(1.0, 0.0, 0.0, 0.0));

    EXPECT_FALSE(aabb.is_finite);
}

TEST(PhysicsBroadphase, FindsOverlappingPairs)
{
    srp::physics::Broadphase broadphase;

    broadphase.upsert(1, srp::physics::Aabb{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, true});
    broadphase.upsert(2, srp::physics::Aabb{{0.5, 0.5, 0.5}, {1.5, 1.5, 1.5}, true});
    broadphase.upsert(3, srp::physics::Aabb{{2.0, 2.0, 2.0}, {3.0, 3.0, 3.0}, true});
    broadphase.upsert(4, srp::physics::Aabb{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, false});

    const auto pairs = broadphase.findOverlappingPairs();
    const auto actual = toSet(pairs);

    const std::set<std::pair<srp::physics::BodyId, srp::physics::BodyId>> expected = {
        {1, 2},
        {1, 4},
        {2, 4},
        {3, 4}};

    EXPECT_EQ(actual, expected);
}
