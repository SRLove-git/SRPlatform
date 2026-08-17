#include "editor/course_catalog.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace srp::editor
{
namespace
{

TEST(CourseCatalogTest, ParsesCourseFields)
{
    const nlohmann::json json = {
        {"id", "rc_car"},
        {"title", "RC 小车"},
        {"entity", "car"},
        {"script", "assets/scripts/courses/rc_car.lua"},
        {"objective", "控制小车前进、停止、转向"},
        {"initial_bodies", nlohmann::json::array({
            nlohmann::json{{"kind", "box"}, {"position", {1.0, 0.5, 0.0}}},
        })},
        {"channels", {"battery_voltage_v", "chassis_speed_m_s"}}};

    std::string error;
    const auto course = parseCourse(json, error);
    ASSERT_TRUE(course.has_value()) << error;
    EXPECT_EQ(course->id, "rc_car");
    EXPECT_EQ(course->entity, "car");
    EXPECT_EQ(course->script, "assets/scripts/courses/rc_car.lua");
    EXPECT_EQ(course->initial_bodies.size(), 1u);
    EXPECT_EQ(course->initial_bodies.front().kind, "box");
    EXPECT_EQ(course->initial_bodies.front().position,
              srp::math::Vec3(1.0, 0.5, 0.0));
    EXPECT_EQ(course->channels.size(), 2u);
}

TEST(CourseCatalogTest, MissingIdFails)
{
    std::string error;
    EXPECT_FALSE(parseCourse(nlohmann::json{{"title", "no id"}}, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(CourseCatalogTest, LoadDirectoryAndFind)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_courses_test";
    std::filesystem::create_directories(directory);

    {
        std::ofstream stream(directory / "a.json");
        stream << nlohmann::json{{"id", "course_a"}}.dump();
    }
    {
        std::ofstream stream(directory / "b.json");
        stream << nlohmann::json{{"id", "course_b"}}.dump();
    }

    CourseCatalog catalog;
    std::string error;
    ASSERT_TRUE(catalog.loadDirectory(directory, error)) << error;
    EXPECT_EQ(catalog.courses().size(), 2u);
    EXPECT_NE(catalog.find("course_a"), nullptr);
    EXPECT_NE(catalog.find("course_b"), nullptr);
    EXPECT_EQ(catalog.find("missing"), nullptr);

    std::filesystem::remove_all(directory);
}

TEST(CourseCatalogTest, MissingDirectoryFails)
{
    CourseCatalog catalog;
    std::string error;
    EXPECT_FALSE(catalog.loadDirectory(
        std::filesystem::temp_directory_path() / "srp_no_such_course_dir",
        error));
    EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace srp::editor
