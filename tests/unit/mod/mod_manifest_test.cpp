#include "mod/mod_manifest.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace
{

nlohmann::json validManifestJson()
{
    return nlohmann::json{
        {"id", "com.example.rc_car"},
        {"name", "RC Car Pack"},
        {"version", "1.0.0"},
        {"description", "An example RC car mod."},
        {"author", "SRPlatform"},
        {"entry", "scripts/main.lua"},
        {"blueprint", "blueprint.json"},
        {"requires", nlohmann::json::array({"com.example.core"})}};
}

}  // namespace

TEST(ModManifest, ParsesCompleteManifest)
{
    std::string error;
    const auto manifest =
        srp::mod::parseManifest(validManifestJson(), error);

    ASSERT_TRUE(manifest.has_value());
    EXPECT_EQ(manifest->id, "com.example.rc_car");
    EXPECT_EQ(manifest->name, "RC Car Pack");
    EXPECT_EQ(manifest->version, "1.0.0");
    EXPECT_EQ(manifest->description, "An example RC car mod.");
    EXPECT_EQ(manifest->author, "SRPlatform");
    EXPECT_EQ(manifest->entry, "scripts/main.lua");
    EXPECT_EQ(manifest->blueprint, "blueprint.json");
    ASSERT_EQ(manifest->dependencies.size(), 1U);
    EXPECT_EQ(manifest->dependencies[0], "com.example.core");
    EXPECT_TRUE(error.empty());
}

TEST(ModManifest, ParsesMinimalManifest)
{
    const nlohmann::json json = {
        {"id", "minimal.mod"},
        {"name", "Minimal"},
        {"version", "0.1.0"},
        {"entry", "main.lua"}};
    std::string error;

    const auto manifest = srp::mod::parseManifest(json, error);

    ASSERT_TRUE(manifest.has_value());
    EXPECT_EQ(manifest->id, "minimal.mod");
    EXPECT_TRUE(manifest->description.empty());
    EXPECT_TRUE(manifest->author.empty());
    EXPECT_TRUE(manifest->blueprint.empty());
    EXPECT_TRUE(manifest->dependencies.empty());
}

TEST(ModManifest, RejectsNonObject)
{
    std::string error;

    EXPECT_FALSE(srp::mod::parseManifest(nlohmann::json::array(), error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(ModManifest, RejectsMissingRequiredFields)
{
    const std::vector<std::string> keys = {"id", "name", "version", "entry"};

    for (const std::string& key : keys)
    {
        nlohmann::json json = validManifestJson();
        json.erase(key);
        std::string error;

        EXPECT_FALSE(srp::mod::parseManifest(json, error).has_value())
            << "missing field: " << key;
        EXPECT_FALSE(error.empty());
    }
}

TEST(ModManifest, RejectsEmptyRequiredFields)
{
    nlohmann::json json = validManifestJson();
    json["id"] = "";
    std::string error;
    EXPECT_FALSE(srp::mod::parseManifest(json, error).has_value());

    json = validManifestJson();
    json["version"] = "";
    error.clear();
    EXPECT_FALSE(srp::mod::parseManifest(json, error).has_value());
}

TEST(ModManifest, RejectsInvalidModIds)
{
    const std::vector<std::string> invalid_ids = {
        ".leading.dot",
        "-leading-dash",
        "_leading_underscore",
        "has space",
        "中文id",
        "com/example"};

    for (const std::string& id : invalid_ids)
    {
        nlohmann::json json = validManifestJson();
        json["id"] = id;
        std::string error;

        EXPECT_FALSE(srp::mod::parseManifest(json, error).has_value())
            << "invalid id: " << id;
    }
}

TEST(ModManifest, RejectsWrongFieldTypes)
{
    nlohmann::json json = validManifestJson();
    json["name"] = 42;
    std::string error;
    EXPECT_FALSE(srp::mod::parseManifest(json, error).has_value());

    json = validManifestJson();
    json["requires"] = "com.example.core";
    error.clear();
    EXPECT_FALSE(srp::mod::parseManifest(json, error).has_value());
}

TEST(ModManifest, RejectsInvalidRequiresEntries)
{
    nlohmann::json json = validManifestJson();
    json["requires"] = nlohmann::json::array({"ok.id", "bad id"});
    std::string error;

    EXPECT_FALSE(srp::mod::parseManifest(json, error).has_value());
}

TEST(ModManifest, LoadsManifestFromFile)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_mod_manifest_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path path = directory / "mod.json";

    {
        std::ofstream stream(path);
        stream << validManifestJson().dump(4);
    }

    std::string error;
    const auto manifest = srp::mod::loadManifest(path, error);

    ASSERT_TRUE(manifest.has_value()) << error;
    EXPECT_EQ(manifest->id, "com.example.rc_car");

    std::filesystem::remove_all(directory);
}

TEST(ModManifest, ReportsMissingFile)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "srp_mod_missing" / "mod.json";
    std::string error;

    EXPECT_FALSE(srp::mod::loadManifest(path, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(ModManifest, ReportsInvalidJsonFile)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_mod_invalid_json";
    std::filesystem::create_directories(directory);
    const std::filesystem::path path = directory / "mod.json";

    {
        std::ofstream stream(path);
        stream << "{ not valid json";
    }

    std::string error;
    EXPECT_FALSE(srp::mod::loadManifest(path, error).has_value());
    EXPECT_FALSE(error.empty());

    std::filesystem::remove_all(directory);
}
