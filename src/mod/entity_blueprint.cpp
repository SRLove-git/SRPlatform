#include "mod/entity_blueprint.hpp"

namespace srp::mod
{

std::optional<EntityBlueprint> parseEntityBlueprint(
    const nlohmann::json& json,
    std::string& error)
{
    error.clear();

    if (!json.is_object())
    {
        error = "entity blueprint must be a JSON object";
        return std::nullopt;
    }

    EntityBlueprint blueprint;

    const auto id_it = json.find("id");
    if (id_it == json.end() ||
        !id_it->is_string() ||
        id_it->get<std::string>().empty())
    {
        error = "entity blueprint requires a non-empty string 'id'";
        return std::nullopt;
    }
    blueprint.id = id_it->get<std::string>();

    const auto kind_it = json.find("kind");
    if (kind_it == json.end() || !kind_it->is_string())
    {
        error = "entity blueprint requires a string 'kind'";
        return std::nullopt;
    }
    blueprint.kind = kind_it->get<std::string>();
    if (blueprint.kind != "car" && blueprint.kind != "drone")
    {
        error = "unsupported entity kind: " + blueprint.kind;
        return std::nullopt;
    }

    if (const auto parameters_it = json.find("parameters");
        parameters_it != json.end())
    {
        if (!parameters_it->is_object())
        {
            error = "entity blueprint 'parameters' must be an object";
            return std::nullopt;
        }
        blueprint.parameters = *parameters_it;
    }

    return blueprint;
}

}  // namespace srp::mod
