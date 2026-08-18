#include "physics/physics_world.hpp"

#include <cstring>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace srp::physics
{
namespace
{

struct WorldSnapshot
{
    std::vector<math::Vec3> positions;
    std::vector<math::Quat> orientations;
    std::vector<math::Vec3> linear_velocities;
    std::vector<math::Vec3> angular_velocities;
    std::vector<math::Vec3> contact_points;
    std::vector<math::Vec3> contact_normals;
    std::vector<double> contact_penetrations;
    std::vector<double> contact_impulses;
};

std::uint64_t bits(double value)
{
    std::uint64_t result = 0;
    std::memcpy(&result, &value, sizeof(value));
    return result;
}

bool bitEqual(double left, double right)
{
    return bits(left) == bits(right);
}

WorldSnapshot capture(const PhysicsWorld& world)
{
    WorldSnapshot snapshot;
    for (const BodyId id : world.bodyIds())
    {
        const RigidBodyState* body = world.body(id);
        if (body == nullptr)
        {
            continue;
        }
        snapshot.positions.push_back(body->position);
        snapshot.orientations.push_back(body->orientation);
        snapshot.linear_velocities.push_back(body->linear_velocity);
        snapshot.angular_velocities.push_back(body->angular_velocity);
    }
    for (const Contact& contact : world.contacts())
    {
        snapshot.contact_points.push_back(contact.point.point);
        snapshot.contact_normals.push_back(contact.point.normal);
        snapshot.contact_penetrations.push_back(contact.point.penetration);
        snapshot.contact_impulses.push_back(contact.point.normal_impulse);
    }
    return snapshot;
}

void expectSnapshotEqual(const WorldSnapshot& left, const WorldSnapshot& right)
{
    ASSERT_EQ(left.positions.size(), right.positions.size());
    ASSERT_EQ(left.orientations.size(), right.orientations.size());
    ASSERT_EQ(left.linear_velocities.size(), right.linear_velocities.size());
    ASSERT_EQ(left.angular_velocities.size(), right.angular_velocities.size());
    ASSERT_EQ(left.contact_points.size(), right.contact_points.size());
    ASSERT_EQ(left.contact_impulses.size(), right.contact_impulses.size());

    for (std::size_t i = 0; i < left.positions.size(); ++i)
    {
        EXPECT_TRUE(bitEqual(left.positions[i].x, right.positions[i].x));
        EXPECT_TRUE(bitEqual(left.positions[i].y, right.positions[i].y));
        EXPECT_TRUE(bitEqual(left.positions[i].z, right.positions[i].z));
        EXPECT_TRUE(bitEqual(left.orientations[i].w, right.orientations[i].w));
        EXPECT_TRUE(bitEqual(left.orientations[i].x, right.orientations[i].x));
        EXPECT_TRUE(bitEqual(left.orientations[i].y, right.orientations[i].y));
        EXPECT_TRUE(bitEqual(left.orientations[i].z, right.orientations[i].z));
        EXPECT_TRUE(bitEqual(left.linear_velocities[i].x, right.linear_velocities[i].x));
        EXPECT_TRUE(bitEqual(left.linear_velocities[i].y, right.linear_velocities[i].y));
        EXPECT_TRUE(bitEqual(left.linear_velocities[i].z, right.linear_velocities[i].z));
        EXPECT_TRUE(bitEqual(left.angular_velocities[i].x, right.angular_velocities[i].x));
        EXPECT_TRUE(bitEqual(left.angular_velocities[i].y, right.angular_velocities[i].y));
        EXPECT_TRUE(bitEqual(left.angular_velocities[i].z, right.angular_velocities[i].z));
    }

    for (std::size_t i = 0; i < left.contact_points.size(); ++i)
    {
        EXPECT_TRUE(bitEqual(left.contact_points[i].x, right.contact_points[i].x));
        EXPECT_TRUE(bitEqual(left.contact_points[i].y, right.contact_points[i].y));
        EXPECT_TRUE(bitEqual(left.contact_points[i].z, right.contact_points[i].z));
        EXPECT_TRUE(bitEqual(left.contact_normals[i].x, right.contact_normals[i].x));
        EXPECT_TRUE(bitEqual(left.contact_normals[i].y, right.contact_normals[i].y));
        EXPECT_TRUE(bitEqual(left.contact_normals[i].z, right.contact_normals[i].z));
        EXPECT_TRUE(bitEqual(left.contact_penetrations[i], right.contact_penetrations[i]));
        EXPECT_TRUE(bitEqual(left.contact_impulses[i], right.contact_impulses[i]));
    }
}

void restore(PhysicsWorld& world, const WorldSnapshot& snapshot)
{
    std::size_t index = 0;
    for (const BodyId id : world.bodyIds())
    {
        RigidBodyState* body = world.body(id);
        if (body == nullptr)
        {
            continue;
        }
        body->position = snapshot.positions[index];
        body->orientation = snapshot.orientations[index];
        body->linear_velocity = snapshot.linear_velocities[index];
        body->angular_velocity = snapshot.angular_velocities[index];
        ++index;
    }
}

PhysicsWorld buildScene()
{
    PhysicsWorld world;

    RigidBodyState ground_state;
    ground_state.type = RigidBodyType::kStatic;
    PlaneShape ground;
    ground.normal = math::Vec3(0.0, 1.0, 0.0);
    world.createBody(ground_state, ground);

    for (int i = 0; i < 8; ++i)
    {
        RigidBodyState state;
        state.type = RigidBodyType::kDynamic;
        state.mass = 1.0;
        state.position = math::Vec3(
            static_cast<double>(i % 4) * 1.1 - 1.5,
            2.0 + static_cast<double>(i % 3) * 1.5,
            static_cast<double>(i / 4) * 1.3 - 0.5);
        state.linear_velocity = math::Vec3(
            0.3 * static_cast<double>(i % 2),
            -0.5,
            0.2 * static_cast<double>(i % 3));
        state.restitution = 0.2;
        state.friction = 0.6;
        BoxShape box;
        box.half_extents = math::Vec3(0.4 + 0.05 * i);
        world.createBody(state, box);
    }

    for (int i = 0; i < 4; ++i)
    {
        RigidBodyState state;
        state.type = RigidBodyType::kDynamic;
        state.mass = 0.5;
        state.position = math::Vec3(
            2.5 + static_cast<double>(i) * 0.7,
            3.0 + static_cast<double>(i) * 1.2,
            1.0);
        state.linear_velocity = math::Vec3(-0.4, -0.3, 0.1);
        SphereShape sphere;
        sphere.radius = 0.3;
        world.createBody(state, sphere);
    }

    return world;
}

TEST(PhysicsDeterminismTest, IdenticalBuildsProduceIdenticalResults)
{
    PhysicsWorld first = buildScene();
    PhysicsWorld second = buildScene();

    for (int i = 0; i < 300; ++i)
    {
        first.step(1.0 / 60.0);
        second.step(1.0 / 60.0);
    }

    expectSnapshotEqual(capture(first), capture(second));
}

TEST(PhysicsDeterminismTest, ReplayingFromInitialStateIsBitIdentical)
{
    PhysicsWorld world = buildScene();
    const WorldSnapshot initial = capture(world);

    for (int i = 0; i < 300; ++i)
    {
        world.step(1.0 / 60.0);
    }
    const WorldSnapshot first_run = capture(world);

    restore(world, initial);
    for (int i = 0; i < 300; ++i)
    {
        world.step(1.0 / 60.0);
    }
    const WorldSnapshot second_run = capture(world);

    expectSnapshotEqual(first_run, second_run);
}

}  // namespace
}  // namespace srp::physics
