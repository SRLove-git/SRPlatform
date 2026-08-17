#include "physics/physics_world.hpp"

#include <memory>

#include <gtest/gtest.h>

namespace
{

struct Scene
{
    std::unique_ptr<srp::physics::PhysicsWorld> world;
    srp::physics::BodyId ground{srp::physics::kInvalidBodyId};
    srp::physics::BodyId sphere{srp::physics::kInvalidBodyId};
};

Scene makeScene()
{
    Scene scene;
    scene.world = std::make_unique<srp::physics::PhysicsWorld>();
    scene.world->setGravity(srp::math::Vec3(0.0, -9.81, 0.0));

    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;

    srp::physics::PlaneShape plane;
    plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    scene.ground = scene.world->createBody(ground_state, plane);

    srp::physics::RigidBodyState sphere_state;
    sphere_state.type = srp::physics::RigidBodyType::kDynamic;
    sphere_state.mass = 1.0;
    sphere_state.position = srp::math::Vec3(0.0, 2.0, 0.0);

    srp::physics::SphereShape sphere_shape;
    sphere_shape.radius = 0.5;
    scene.sphere = scene.world->createBody(sphere_state, sphere_shape);

    return scene;
}

}  // namespace

TEST(PhysicsRegression, FreeFallMatchesSemiImplicitEuler)
{
    srp::physics::PhysicsWorld world;
    world.setGravity(srp::math::Vec3(0.0, -9.81, 0.0));

    srp::physics::RigidBodyState state;
    state.type = srp::physics::RigidBodyType::kDynamic;
    state.mass = 1.0;
    const srp::physics::BodyId id = world.createBody(state);

    constexpr int kSteps = 120;
    constexpr double kDt = 1.0 / 60.0;
    for (int i = 0; i < kSteps; ++i)
    {
        world.step(kDt);
    }

    const srp::physics::RigidBodyState* body = world.body(id);
    ASSERT_NE(body, nullptr);

    const double expected_velocity_y = -9.81 * kSteps * kDt;
    const double expected_position_y =
        -0.5 * 9.81 * kDt * kDt * kSteps * (kSteps + 1);

    EXPECT_NEAR(body->linear_velocity.y, expected_velocity_y, 1e-12);
    EXPECT_NEAR(body->position.y, expected_position_y, 1e-12);
}

TEST(PhysicsRegression, SameInitialStateProducesSameResult)
{
    Scene scene_a = makeScene();
    Scene scene_b = makeScene();

    constexpr int kSteps = 300;
    constexpr double kDt = 1.0 / 60.0;

    for (int i = 0; i < kSteps; ++i)
    {
        scene_a.world->step(kDt);
        scene_b.world->step(kDt);
    }

    const srp::physics::RigidBodyState* sphere_a = scene_a.world->body(scene_a.sphere);
    const srp::physics::RigidBodyState* sphere_b = scene_b.world->body(scene_b.sphere);
    ASSERT_NE(sphere_a, nullptr);
    ASSERT_NE(sphere_b, nullptr);

    EXPECT_DOUBLE_EQ(sphere_a->position.x, sphere_b->position.x);
    EXPECT_DOUBLE_EQ(sphere_a->position.y, sphere_b->position.y);
    EXPECT_DOUBLE_EQ(sphere_a->position.z, sphere_b->position.z);
    EXPECT_DOUBLE_EQ(sphere_a->linear_velocity.x, sphere_b->linear_velocity.x);
    EXPECT_DOUBLE_EQ(sphere_a->linear_velocity.y, sphere_b->linear_velocity.y);
    EXPECT_DOUBLE_EQ(sphere_a->linear_velocity.z, sphere_b->linear_velocity.z);
}
