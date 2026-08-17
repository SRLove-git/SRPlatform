#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace srp::mod
{

// Manifest of a single mod package. Path fields are relative to the mod
// root directory. See docs/mod-manifest.md for the full format.
struct ModManifest
{
    std::string id;
    std::string name;
    std::string version;
    std::string description;
    std::string author;
    std::string entry;
    // JSON field name is "requires"; "requires" is a C++20 keyword.
    std::vector<std::string> dependencies;
};

// Parses and validates a mod.json value. On failure returns nullopt and
// writes a human-readable reason into error.
std::optional<ModManifest> parseManifest(
    const nlohmann::json& json,
    std::string& error);

// Reads a mod.json file from disk and validates it.
std::optional<ModManifest> loadManifest(
    const std::filesystem::path& path,
    std::string& error);

}  // namespace srp::mod
