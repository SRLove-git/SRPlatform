#include "bridge/distance_sensor_model.hpp"

#include "physics/physics_world.hpp"

#include <gtest/gtest.h>

#include <glm/gtc/quaternion.hpp>

namespace
{

constexpr double kPi = srp::math::kPi;

srp::physics::PhysicsWorld worldWithBox(
    const srp::math::Vec3& position,
    const srp::math::Vec3& half_extents)
{
    srp::physics::PhysicsWorld world;
    srp::physics::RigidBodyState state;
    state.position = position;
    srp::physics::BoxShape shape;
    shape.half_extents = half_extents;
    world.createBody(state, shape);
    return world;
}

}  // namespace

TEST(BridgeDistanceSensor, NoDetectionReportsMaxRange)
{
    srp::bridge::DistanceSensorModel sensor;
    srp::physics::PhysicsWorld world;

    sensor.update(world);

    EXPECT_FALSE(sensor.detected());
    EXPECT_DOUBLE_EQ(sensor.distance(), 4.0);
}

TEST(BridgeDistanceSensor, DownwardBeamMeasuresBoxTop)
{
    srp::bridge::DistanceSensorModel sensor;
    sensor.setPose(srp::math::Vec3(0.0, 2.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    srp::physics::PhysicsWorld world =
        worldWithBox(srp::math::Vec3(0.0, 0.5, 0.0), srp::math::Vec3(0.5));

    sensor.update(world);

    EXPECT_TRUE(sensor.detected());
    EXPECT_NEAR(sensor.distance(), 1.0, 1e-12);
}

TEST(BridgeDistanceSensor, MeasuresSphereSurface)
{
    srp::bridge::DistanceSensorModel sensor;
    sensor.setPose(srp::math::Vec3(0.0, 3.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));

    srp::physics::PhysicsWorld world;
    srp::physics::RigidBodyState state;
    state.position = srp::math::Vec3(0.0, 1.0, 0.0);
    srp::physics::SphereShape sphere;
    sphere.radius = 0.5;
    world.createBody(state, sphere);

    sensor.update(world);

    EXPECT_TRUE(sensor.detected());
    EXPECT_NEAR(sensor.distance(), 1.5, 1e-12);
}

TEST(BridgeDistanceSensor, MeasuresGroundPlane)
{
    srp::bridge::DistanceSensorModel sensor;
    sensor.setPose(srp::math::Vec3(0.0, 2.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));

    srp::physics::PhysicsWorld world;
    srp::physics::RigidBodyState state;
    state.position = srp::math::Vec3(0.0);
    srp::physics::PlaneShape plane;
    plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    world.createBody(state, plane);

    sensor.update(world);

    EXPECT_TRUE(sensor.detected());
    EXPECT_NEAR(sensor.distance(), 2.0, 1e-12);
}

TEST(BridgeDistanceSensor, RotatedPoseAimsBeam)
{
    srp::bridge::DistanceSensorParameters parameters;
    parameters.beam_axis_local = srp::math::Vec3(0.0, 0.0, 1.0);
    srp::bridge::DistanceSensorModel sensor(parameters);
    sensor.setPose(
        srp::math::Vec3(0.0, 0.0, 0.0),
        glm::angleAxis(kPi / 2.0, srp::math::Vec3(0.0, 1.0, 0.0)));

    srp::physics::PhysicsWorld world =
        worldWithBox(srp::math::Vec3(1.5, 0.0, 0.0), srp::math::Vec3(0.5));

    sensor.update(world);

    EXPECT_NEAR(sensor.beamDirectionWorld().x, 1.0, 1e-12);
    EXPECT_TRUE(sensor.detected());
    EXPECT_NEAR(sensor.distance(), 1.0, 1e-12);
}

TEST(BridgeDistanceSensor, OrientedPoseTurnsBeamDownward)
{
    srp::bridge::DistanceSensorParameters parameters;
    parameters.beam_axis_local = srp::math::Vec3(0.0, 0.0, 1.0);
    srp::bridge::DistanceSensorModel sensor(parameters);
    sensor.setPose(
        srp::math::Vec3(0.0, 2.0, 0.0),
        glm::angleAxis(kPi / 2.0, srp::math::Vec3(1.0, 0.0, 0.0)));

    srp::physics::PhysicsWorld world;
    srp::physics::RigidBodyState state;
    srp::physics::PlaneShape plane;
    plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    world.createBody(state, plane);

    sensor.update(world);

    EXPECT_NEAR(sensor.beamDirectionWorld().y, -1.0, 1e-12);
    EXPECT_TRUE(sensor.detected());
    EXPECT_NEAR(sensor.distance(), 2.0, 1e-12);
}

TEST(BridgeDistanceSensor, ReportsNearestObject)
{
    srp::bridge::DistanceSensorModel sensor;
    sensor.setPose(srp::math::Vec3(0.0, 3.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));

    srp::physics::PhysicsWorld world;
    srp::physics::RigidBodyState lower_state;
    lower_state.position = srp::math::Vec3(0.0, 0.5, 0.0);
    srp::physics::BoxShape lower_box;
    lower_box.half_extents = srp::math::Vec3(0.5);
    world.createBody(lower_state, lower_box);

    srp::physics::RigidBodyState upper_state;
    upper_state.position = srp::math::Vec3(0.0, 2.0, 0.0);
    srp::physics::BoxShape upper_box;
    upper_box.half_extents = srp::math::Vec3(0.5);
    world.createBody(upper_state, upper_box);

    sensor.update(world);

    EXPECT_TRUE(sensor.detected());
    EXPECT_NEAR(sensor.distance(), 0.5, 1e-12);
}

TEST(BridgeDistanceSensor, ClampsReadingWhenObjectOutOfRange)
{
    srp::bridge::DistanceSensorParameters parameters;
    parameters.max_range_m = 1.0;
    srp::bridge::DistanceSensorModel sensor(parameters);
    sensor.setPose(srp::math::Vec3(0.0, 3.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    srp::physics::PhysicsWorld world =
        worldWithBox(srp::math::Vec3(0.0, 0.5, 0.0), srp::math::Vec3(0.5));

    sensor.update(world);

    EXPECT_FALSE(sensor.detected());
    EXPECT_DOUBLE_EQ(sensor.distance(), 1.0);
}

TEST(BridgeDistanceSensor, SensorInsideObjectReportsZero)
{
    srp::bridge::DistanceSensorModel sensor;
    sensor.setPose(srp::math::Vec3(0.0, 0.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    srp::physics::PhysicsWorld world =
        worldWithBox(srp::math::Vec3(0.0, 0.0, 0.0), srp::math::Vec3(0.5));

    sensor.update(world);

    EXPECT_TRUE(sensor.detected());
    EXPECT_DOUBLE_EQ(sensor.distance(), 0.0);
}

TEST(BridgeDistanceSensor, MeasuresCylinderCapAndSide)
{
    srp::physics::PhysicsWorld world;
    srp::physics::RigidBodyState state;
    srp::physics::CylinderShape cylinder;
    cylinder.half_height = 0.5;
    cylinder.radius = 0.5;
    world.createBody(state, cylinder);

    srp::bridge::DistanceSensorModel cap_sensor;
    cap_sensor.setPose(srp::math::Vec3(0.0, 2.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    cap_sensor.update(world);
    EXPECT_TRUE(cap_sensor.detected());
    EXPECT_NEAR(cap_sensor.distance(), 1.5, 1e-12);

    srp::bridge::DistanceSensorParameters side_parameters;
    side_parameters.beam_axis_local = srp::math::Vec3(-1.0, 0.0, 0.0);
    srp::bridge::DistanceSensorModel side_sensor(side_parameters);
    side_sensor.setPose(srp::math::Vec3(2.0, 0.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    side_sensor.update(world);
    EXPECT_TRUE(side_sensor.detected());
    EXPECT_NEAR(side_sensor.distance(), 1.5, 1e-12);
}

TEST(BridgeDistanceSensor, ConvexHullUsesBoundingSphere)
{
    srp::physics::PhysicsWorld world;
    srp::physics::RigidBodyState state;
    state.position = srp::math::Vec3(0.0, 1.5, 0.0);
    srp::physics::ConvexHullShape hull;
    hull.points = {
        srp::math::Vec3(-0.5, -0.5, -0.5),
        srp::math::Vec3(0.5, -0.5, -0.5),
        srp::math::Vec3(-0.5, 0.5, -0.5),
        srp::math::Vec3(0.5, 0.5, -0.5),
        srp::math::Vec3(-0.5, -0.5, 0.5),
        srp::math::Vec3(0.5, -0.5, 0.5),
        srp::math::Vec3(-0.5, 0.5, 0.5),
        srp::math::Vec3(0.5, 0.5, 0.5)};
    world.createBody(state, hull);

    srp::bridge::DistanceSensorModel sensor;
    sensor.setPose(srp::math::Vec3(0.0, 3.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    sensor.update(world);

    const double radius = std::sqrt(3.0) / 2.0;
    EXPECT_TRUE(sensor.detected());
    EXPECT_NEAR(sensor.distance(), 3.0 - (1.5 + radius), 1e-12);
}

TEST(BridgeDistanceSensor, ResetClearsDetection)
{
    srp::bridge::DistanceSensorModel sensor;
    sensor.setPose(srp::math::Vec3(0.0, 2.0, 0.0), srp::math::Quat(1.0, 0.0, 0.0, 0.0));
    srp::physics::PhysicsWorld world =
        worldWithBox(srp::math::Vec3(0.0, 0.5, 0.0), srp::math::Vec3(0.5));
    sensor.update(world);
    ASSERT_TRUE(sensor.detected());

    sensor.reset();

    EXPECT_FALSE(sensor.detected());
    EXPECT_DOUBLE_EQ(sensor.distance(), 4.0);
}

TEST(BridgeDistanceSensor, RejectsInvalidParameters)
{
    srp::bridge::DistanceSensorParameters parameters;
    parameters.max_range_m = 0.0;
    EXPECT_THROW(
        srp::bridge::DistanceSensorModel{parameters},
        std::invalid_argument);

    parameters = srp::bridge::DistanceSensorParameters{};
    parameters.beam_axis_local = srp::math::Vec3(0.0);
    EXPECT_THROW(
        srp::bridge::DistanceSensorModel{parameters},
        std::invalid_argument);
}
