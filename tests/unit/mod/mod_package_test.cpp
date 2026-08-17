#include "mod/mod_package.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace
{

std::filesystem::path createModDirectory(
    const std::string& name,
    bool with_entry)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_mod_package_test" / name;
    std::filesystem::create_directories(directory / "scripts");

    nlohmann::json manifest;
    manifest["id"] = "com.example." + name;
    manifest["name"] = name;
    manifest["version"] = "1.0.0";
    manifest["entry"] = "scripts/main.lua";
    std::ofstream(directory / "mod.json") << manifest.dump(2);

    if (with_entry)
    {
        std::ofstream(directory / "scripts" / "main.lua")
            << "function update(dt) end\n";
    }
    return directory;
}

}  // namespace

TEST(ModPackage, LoadsManifestAndResolvesEntry)
{
    const std::filesystem::path directory =
        createModDirectory("package_a", true);

    std::string error;
    const auto package = srp::mod::loadModPackage(directory, error);

    ASSERT_TRUE(package.has_value()) << error;
    EXPECT_EQ(package->manifest.id, "com.example.package_a");
    EXPECT_EQ(package->root, directory);
    EXPECT_EQ(package->entryPath(), directory / "scripts" / "main.lua");
    EXPECT_TRUE(std::filesystem::exists(package->entryPath()));

    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_mod_package_test");
}

TEST(ModPackage, ReportsMissingManifest)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        "srp_mod_package_missing_manifest";
    std::filesystem::create_directories(directory);

    std::string error;
    EXPECT_FALSE(srp::mod::loadModPackage(directory, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(ModPackage, ReportsMissingEntryScript)
{
    const std::filesystem::path directory =
        createModDirectory("package_no_entry", false);

    std::string error;
    EXPECT_FALSE(srp::mod::loadModPackage(directory, error).has_value());
    EXPECT_FALSE(error.empty());

    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_mod_package_test");
}
