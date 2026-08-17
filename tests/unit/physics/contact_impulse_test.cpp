#include "physics/physics_world.hpp"

#include <gtest/gtest.h>

namespace srp::physics
{
namespace
{

TEST(ContactImpulseTest, DroppedBoxProducesContactImpulse)
{
    PhysicsWorld world;

    RigidBodyState ground_state;
    ground_state.type = RigidBodyType::kStatic;
    PlaneShape ground;
    ground.normal = math::Vec3(0.0, 1.0, 0.0);
    world.createBody(ground_state, ground);

    RigidBodyState box_state;
    box_state.type = RigidBodyType::kDynamic;
    box_state.mass = 1.0;
    box_state.position = math::Vec3(0.0, 1.0, 0.0);
    box_state.linear_velocity = math::Vec3(0.0, -2.0, 0.0);
    BoxShape box;
    box.half_extents = math::Vec3(0.5);
    world.createBody(box_state, box);

    bool contacted = false;
    for (int i = 0; i < 120 && !contacted; ++i)
    {
        world.step(1.0 / 60.0);
        contacted = !world.contacts().empty();
    }

    ASSERT_TRUE(contacted);
    bool found_impulse = false;
    for (const Contact& contact : world.contacts())
    {
        if (contact.point.normal_impulse > 0.0)
        {
            found_impulse = true;
        }
    }
    EXPECT_TRUE(found_impulse);
}

TEST(ContactImpulseTest, RestingContactStillReportsImpulse)
{
    PhysicsWorld world;

    RigidBodyState ground_state;
    ground_state.type = RigidBodyType::kStatic;
    PlaneShape ground;
    ground.normal = math::Vec3(0.0, 1.0, 0.0);
    world.createBody(ground_state, ground);

    RigidBodyState box_state;
    box_state.type = RigidBodyType::kDynamic;
    box_state.mass = 1.0;
    box_state.position = math::Vec3(0.0, 0.6, 0.0);
    BoxShape box;
    box.half_extents = math::Vec3(0.5);
    world.createBody(box_state, box);

    for (int i = 0; i < 10; ++i)
    {
        world.step(1.0 / 60.0);
    }

    ASSERT_FALSE(world.contacts().empty());
    EXPECT_GT(world.contacts().front().point.normal_impulse, 0.0);
}

}  // namespace
}  // namespace srp::physics
