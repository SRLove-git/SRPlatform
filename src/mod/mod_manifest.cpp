#include "mod/mod_manifest.hpp"

#include <cctype>
#include <fstream>

namespace srp::mod
{
namespace
{

bool isValidModId(const std::string& id)
{
    if (id.empty())
    {
        return false;
    }

    if (id.front() == '.' || id.front() == '-' || id.front() == '_')
    {
        return false;
    }

    for (const unsigned char character : id)
    {
        if (!(std::isalnum(character) ||
              character == '.' ||
              character == '_' ||
              character == '-'))
        {
            return false;
        }
    }

    return true;
}

bool readRequiredString(
    const nlohmann::json& root,
    const char* key,
    std::string& output,
    std::string& error)
{
    const auto it = root.find(key);
    if (it == root.end() || it->is_null())
    {
        error = std::string("missing required field '") + key + "'";
        return false;
    }

    if (!it->is_string())
    {
        error = std::string("field '") + key + "' must be a string";
        return false;
    }

    output = it->get<std::string>();
    if (output.empty())
    {
        error = std::string("field '") + key + "' cannot be empty";
        return false;
    }

    return true;
}

void readOptionalString(
    const nlohmann::json& root,
    const char* key,
    std::string& output)
{
    const auto it = root.find(key);
    if (it != root.end() && it->is_string())
    {
        output = it->get<std::string>();
    }
}

}  // namespace

std::optional<ModManifest> parseManifest(
    const nlohmann::json& json,
    std::string& error)
{
    error.clear();

    if (!json.is_object())
    {
        error = "mod.json must be a JSON object";
        return std::nullopt;
    }

    ModManifest manifest;

    if (!readRequiredString(json, "id", manifest.id, error))
    {
        return std::nullopt;
    }
    if (!isValidModId(manifest.id))
    {
        error = "mod id may only contain letters, digits, '.', '_' and '-', "
                "and must not start with '.', '_' or '-'";
        return std::nullopt;
    }

    if (!readRequiredString(json, "name", manifest.name, error))
    {
        return std::nullopt;
    }

    if (!readRequiredString(json, "version", manifest.version, error))
    {
        return std::nullopt;
    }

    if (!readRequiredString(json, "entry", manifest.entry, error))
    {
        return std::nullopt;
    }

    readOptionalString(json, "description", manifest.description);
    readOptionalString(json, "author", manifest.author);

    const auto requires_it = json.find("requires");
    if (requires_it != json.end() && !requires_it->is_null())
    {
        if (!requires_it->is_array())
        {
            error = "field 'requires' must be an array of mod ids";
            return std::nullopt;
        }

        for (const nlohmann::json& dependency : *requires_it)
        {
            if (!dependency.is_string())
            {
                error = "field 'requires' entries must be strings";
                return std::nullopt;
            }

            const std::string dependency_id = dependency.get<std::string>();
            if (!isValidModId(dependency_id))
            {
                error = "field 'requires' contains an invalid mod id";
                return std::nullopt;
            }
            manifest.dependencies.push_back(dependency_id);
        }
    }

    return manifest;
}

std::optional<ModManifest> loadManifest(
    const std::filesystem::path& path,
    std::string& error)
{
    error.clear();

    std::ifstream stream(path);
    if (!stream.is_open())
    {
        error = "cannot open mod.json: " + path.string();
        return std::nullopt;
    }

    nlohmann::json json;
    try
    {
        stream >> json;
    }
    catch (const nlohmann::json::parse_error& parse_error)
    {
        error = std::string("invalid JSON in mod.json: ") + parse_error.what();
        return std::nullopt;
    }

    return parseManifest(json, error);
}

}  // namespace srp::mod
