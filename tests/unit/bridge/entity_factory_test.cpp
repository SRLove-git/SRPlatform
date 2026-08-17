#include "bridge/entity_factory.hpp"

#include "bridge/car_entity.hpp"
#include "bridge/drone_entity.hpp"

#include <gtest/gtest.h>

namespace
{

srp::mod::EntityBlueprint blueprint(
    const std::string& kind,
    const nlohmann::json& parameters = nlohmann::json::object())
{
    srp::mod::EntityBlueprint result;
    result.id = "test_entity";
    result.kind = kind;
    result.parameters = parameters;
    return result;
}

}  // namespace

TEST(EntityFactory, CreatesCarWithAppliedParameters)
{
    nlohmann::json parameters;
    parameters["chassis_mass"] = 1.2;
    parameters["wheel_radius"] = 0.25;

    std::string error;
    auto entity = srp::bridge::createEntity(
        blueprint("car", parameters), error);

    ASSERT_NE(entity, nullptr) << error;
    EXPECT_STREQ(entity->kind(), "car");

    auto* car = dynamic_cast<srp::bridge::CarEntity*>(entity.get());
    ASSERT_NE(car, nullptr);
    const auto* chassis = car->chassisBody();
    ASSERT_NE(chassis, nullptr);
    EXPECT_DOUBLE_EQ(chassis->mass, 1.2);

    car->setThrottle(1.0);
    car->step(1.0 / 60.0);
    EXPECT_EQ(car->recorder().size(), 1U);
}

TEST(EntityFactory, CreatesDroneWithAppliedParameters)
{
    nlohmann::json parameters;
    parameters["chassis_mass"] = 0.8;
    parameters["max_rotor_angular_velocity_rad_s"] = 1200.0;
    parameters["quadcopter"]["arm_length_m"] = 0.3;
    parameters["quadcopter"]["propeller"]["diameter_m"] = 0.2;

    std::string error;
    auto entity = srp::bridge::createEntity(
        blueprint("drone", parameters), error);

    ASSERT_NE(entity, nullptr) << error;
    EXPECT_STREQ(entity->kind(), "drone");

    auto* drone = dynamic_cast<srp::bridge::DroneEntity*>(entity.get());
    ASSERT_NE(drone, nullptr);
    const auto* body = drone->body();
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->mass, 0.8);
    EXPECT_DOUBLE_EQ(drone->quadcopter().rotorPosition(0).x, 0.3);

    drone->setThrottle(1.0);
    drone->step(1.0 / 60.0);
    EXPECT_GT(drone->quadcopter().totalThrust(), 0.0);
}

TEST(EntityFactory, UsesDefaultsWhenParametersEmpty)
{
    std::string error;
    auto entity = srp::bridge::createEntity(blueprint("car"), error);

    ASSERT_NE(entity, nullptr) << error;
    auto* car = dynamic_cast<srp::bridge::CarEntity*>(entity.get());
    ASSERT_NE(car, nullptr);
    const auto* chassis = car->chassisBody();
    ASSERT_NE(chassis, nullptr);
    EXPECT_DOUBLE_EQ(chassis->mass, 1.0);
}

TEST(EntityFactory, RejectsUnknownKind)
{
    std::string error;
    auto entity = srp::bridge::createEntity(
        blueprint("hovercraft"), error);

    EXPECT_EQ(entity, nullptr);
    EXPECT_FALSE(error.empty());
}
