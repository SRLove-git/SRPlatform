#include "mod/entity_blueprint.hpp"

#include <gtest/gtest.h>

namespace
{

nlohmann::json validCarBlueprint()
{
    return nlohmann::json{
        {"id", "my_rc_car"},
        {"kind", "car"},
        {"parameters", {{"chassis_mass", 1.2}}}};
}

}  // namespace

TEST(EntityBlueprint, ParsesCarBlueprint)
{
    std::string error;
    const auto blueprint =
        srp::mod::parseEntityBlueprint(validCarBlueprint(), error);

    ASSERT_TRUE(blueprint.has_value()) << error;
    EXPECT_EQ(blueprint->id, "my_rc_car");
    EXPECT_EQ(blueprint->kind, "car");
    EXPECT_TRUE(blueprint->parameters.is_object());
    EXPECT_DOUBLE_EQ(blueprint->parameters["chassis_mass"].get<double>(), 1.2);
}

TEST(EntityBlueprint, ParsesMinimalDroneBlueprint)
{
    const nlohmann::json json = {
        {"id", "hover_bot"},
        {"kind", "drone"}};
    std::string error;

    const auto blueprint = srp::mod::parseEntityBlueprint(json, error);

    ASSERT_TRUE(blueprint.has_value()) << error;
    EXPECT_EQ(blueprint->kind, "drone");
    EXPECT_TRUE(blueprint->parameters.is_null() ||
                blueprint->parameters.empty());
}

TEST(EntityBlueprint, RejectsNonObject)
{
    std::string error;
    EXPECT_FALSE(srp::mod::parseEntityBlueprint(
        nlohmann::json::array(), error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(EntityBlueprint, RejectsMissingOrEmptyId)
{
    nlohmann::json json = validCarBlueprint();
    json.erase("id");
    std::string error;
    EXPECT_FALSE(srp::mod::parseEntityBlueprint(json, error).has_value());

    json = validCarBlueprint();
    json["id"] = "";
    error.clear();
    EXPECT_FALSE(srp::mod::parseEntityBlueprint(json, error).has_value());
}

TEST(EntityBlueprint, RejectsMissingOrUnknownKind)
{
    nlohmann::json json = validCarBlueprint();
    json.erase("kind");
    std::string error;
    EXPECT_FALSE(srp::mod::parseEntityBlueprint(json, error).has_value());

    json = validCarBlueprint();
    json["kind"] = "hovercraft";
    error.clear();
    EXPECT_FALSE(srp::mod::parseEntityBlueprint(json, error).has_value());
}

TEST(EntityBlueprint, RejectsNonObjectParameters)
{
    nlohmann::json json = validCarBlueprint();
    json["parameters"] = nlohmann::json::array({1, 2, 3});
    std::string error;

    EXPECT_FALSE(srp::mod::parseEntityBlueprint(json, error).has_value());
    EXPECT_FALSE(error.empty());
}
