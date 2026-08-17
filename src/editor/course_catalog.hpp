#pragma once

#include "core/math/types.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace srp::editor
{

struct SceneObjectSpec
{
    std::string kind;
    srp::math::Vec3 position{0.0};
};

// Data-only description of a learning course. Entity is one of
// "car" / "drone" / "arm"; script is a path to the Lua controller.
struct CourseDefinition
{
    std::string id;
    std::string title;
    std::string description;
    std::string entity{"car"};
    std::string script;
    std::string objective;
    std::vector<SceneObjectSpec> initial_bodies;
    std::vector<std::string> channels;
};

std::optional<CourseDefinition> parseCourse(
    const nlohmann::json& json,
    std::string& error);

// Loads every *.json course definition in a directory.
class CourseCatalog
{
public:
    bool loadDirectory(
        const std::filesystem::path& directory,
        std::string& error);

    const std::vector<CourseDefinition>& courses() const;
    const CourseDefinition* find(const std::string& id) const;

private:
    std::vector<CourseDefinition> courses_;
};

}  // namespace srp::editor
