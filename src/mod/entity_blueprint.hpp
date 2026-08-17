#pragma once

#include <optional>
#include <string>

#include <nlohmann/json.hpp>

namespace srp::mod
{

// Data-only description of an entity that can be instantiated by the
// bridge's entity factory. The parameter schema depends on the kind; see
// docs/entity-blueprint.md.
struct EntityBlueprint
{
    std::string id;
    std::string kind;
    nlohmann::json parameters;
};

// Parses and validates an entity blueprint. Supported kinds are "car" and
// "drone".
std::optional<EntityBlueprint> parseEntityBlueprint(
    const nlohmann::json& json,
    std::string& error);

}  // namespace srp::mod
