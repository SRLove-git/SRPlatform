#include "editor/course_catalog.hpp"

#include <algorithm>
#include <fstream>
#include <system_error>
#include <utility>

namespace srp::editor
{

std::optional<CourseDefinition> parseCourse(
    const nlohmann::json& json,
    std::string& error)
{
    error.clear();
    if (!json.is_object())
    {
        error = "course must be a JSON object";
        return std::nullopt;
    }

    const auto id_it = json.find("id");
    if (id_it == json.end() || !id_it->is_string())
    {
        error = "course requires a string 'id'";
        return std::nullopt;
    }

    CourseDefinition course;
    course.id = id_it->get<std::string>();
    course.title = json.value("title", course.id);
    course.description = json.value("description", std::string{});
    course.entity = json.value("entity", course.entity);
    course.script = json.value("script", std::string{});
    course.objective = json.value("objective", std::string{});

    if (const auto bodies_it = json.find("initial_bodies");
        bodies_it != json.end() && !bodies_it->is_null())
    {
        if (!bodies_it->is_array())
        {
            error = "course 'initial_bodies' must be an array";
            return std::nullopt;
        }

        for (const nlohmann::json& entry : *bodies_it)
        {
            SceneObjectSpec spec;
            spec.kind = entry.value("kind", std::string{});
            if (entry.contains("position") &&
                entry["position"].is_array() &&
                entry["position"].size() == 3)
            {
                spec.position = srp::math::Vec3(
                    entry["position"][0].get<double>(),
                    entry["position"][1].get<double>(),
                    entry["position"][2].get<double>());
            }
            course.initial_bodies.push_back(std::move(spec));
        }
    }

    if (const auto channels_it = json.find("channels");
        channels_it != json.end() && channels_it->is_array())
    {
        for (const nlohmann::json& entry : *channels_it)
        {
            if (entry.is_string())
            {
                course.channels.push_back(entry.get<std::string>());
            }
        }
    }

    return course;
}

bool CourseCatalog::loadDirectory(
    const std::filesystem::path& directory,
    std::string& error)
{
    error.clear();
    courses_.clear();

    std::error_code ec;
    if (!std::filesystem::is_directory(directory, ec))
    {
        error = "course directory not found: " + directory.string();
        return false;
    }

    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec))
    {
        if (entry.path().extension() == ".json")
        {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());

    for (const std::filesystem::path& path : paths)
    {
        std::ifstream stream(path);
        if (!stream.is_open())
        {
            error = "cannot open course file: " + path.string();
            return false;
        }

        try
        {
            const nlohmann::json json = nlohmann::json::parse(stream);
            std::string parse_error;
            const auto course = parseCourse(json, parse_error);
            if (!course.has_value())
            {
                error = path.string() + ": " + parse_error;
                return false;
            }
            courses_.push_back(*course);
        }
        catch (const std::exception& exception)
        {
            error = path.string() + ": " + exception.what();
            return false;
        }
    }

    return true;
}

const std::vector<CourseDefinition>& CourseCatalog::courses() const
{
    return courses_;
}

const CourseDefinition* CourseCatalog::find(const std::string& id) const
{
    for (const CourseDefinition& course : courses_)
    {
        if (course.id == id)
        {
            return &course;
        }
    }
    return nullptr;
}

}  // namespace srp::editor
