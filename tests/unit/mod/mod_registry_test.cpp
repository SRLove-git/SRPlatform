#include "mod/mod_registry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <initializer_list>

namespace
{

srp::mod::ModManifest makeManifest(
    const std::string& id,
    std::initializer_list<const char*> dependencies = {})
{
    srp::mod::ModManifest manifest;
    manifest.id = id;
    manifest.name = id;
    manifest.version = "1.0.0";
    manifest.entry = "main.lua";
    for (const char* dependency : dependencies)
    {
        manifest.dependencies.emplace_back(dependency);
    }
    return manifest;
}

}  // namespace

TEST(ModRegistry, SingleModWithoutDependenciesIsValid)
{
    const std::vector<srp::mod::ModManifest> manifests = {
        makeManifest("standalone")};

    const auto resolution = srp::mod::resolveModDependencies(manifests);

    EXPECT_TRUE(resolution.ok);
    EXPECT_TRUE(resolution.errors.empty());
    ASSERT_EQ(resolution.load_order.size(), 1U);
    EXPECT_EQ(resolution.load_order[0], "standalone");
}

TEST(ModRegistry, EmptyListIsValid)
{
    const auto resolution =
        srp::mod::resolveModDependencies({});

    EXPECT_TRUE(resolution.ok);
    EXPECT_TRUE(resolution.load_order.empty());
}

TEST(ModRegistry, OrdersDependenciesBeforeDependents)
{
    const std::vector<srp::mod::ModManifest> manifests = {
        makeManifest("app", {"framework"}),
        makeManifest("framework", {"core"}),
        makeManifest("core")};

    const auto resolution = srp::mod::resolveModDependencies(manifests);

    ASSERT_TRUE(resolution.ok) << [&resolution]
    {
        std::string joined;
        for (const std::string& error : resolution.errors)
        {
            joined += error + "\n";
        }
        return joined;
    }();
    ASSERT_EQ(resolution.load_order.size(), 3U);
    EXPECT_EQ(resolution.load_order[0], "core");
    EXPECT_EQ(resolution.load_order[1], "framework");
    EXPECT_EQ(resolution.load_order[2], "app");
}

TEST(ModRegistry, ReportsMissingDependency)
{
    const std::vector<srp::mod::ModManifest> manifests = {
        makeManifest("app", {"missing_lib"})};

    const auto resolution = srp::mod::resolveModDependencies(manifests);

    EXPECT_FALSE(resolution.ok);
    ASSERT_EQ(resolution.errors.size(), 1U);
    EXPECT_NE(resolution.errors[0].find("app"), std::string::npos);
    EXPECT_NE(resolution.errors[0].find("missing_lib"), std::string::npos);
}

TEST(ModRegistry, ReportsDuplicateIds)
{
    const std::vector<srp::mod::ModManifest> manifests = {
        makeManifest("dup.mod"),
        makeManifest("dup.mod")};

    const auto resolution = srp::mod::resolveModDependencies(manifests);

    EXPECT_FALSE(resolution.ok);
    ASSERT_EQ(resolution.errors.size(), 1U);
    EXPECT_NE(resolution.errors[0].find("dup.mod"), std::string::npos);
}

TEST(ModRegistry, ReportsDependencyCycle)
{
    const std::vector<srp::mod::ModManifest> manifests = {
        makeManifest("a", {"b"}),
        makeManifest("b", {"a"})};

    const auto resolution = srp::mod::resolveModDependencies(manifests);

    EXPECT_FALSE(resolution.ok);
    ASSERT_FALSE(resolution.errors.empty());
    EXPECT_NE(
        std::find_if(
            resolution.errors.begin(),
            resolution.errors.end(),
            [](const std::string& error)
            {
                return error.find("cycle") != std::string::npos;
            }),
        resolution.errors.end());
}

TEST(ModRegistry, ReportsSelfDependencyCycle)
{
    const std::vector<srp::mod::ModManifest> manifests = {
        makeManifest("selfish", {"selfish"})};

    const auto resolution = srp::mod::resolveModDependencies(manifests);

    EXPECT_FALSE(resolution.ok);
    ASSERT_FALSE(resolution.errors.empty());
    EXPECT_NE(
        resolution.errors[0].find("cycle"),
        std::string::npos);
}

TEST(ModRegistry, LoadOrderPutsDependencyFirst)
{
    const std::vector<srp::mod::ModManifest> manifests = {
        makeManifest("game", {"physics_mod"}),
        makeManifest("physics_mod")};

    const auto resolution = srp::mod::resolveModDependencies(manifests);

    ASSERT_TRUE(resolution.ok);
    ASSERT_EQ(resolution.load_order.size(), 2U);
    const auto physics_position = std::find(
        resolution.load_order.begin(),
        resolution.load_order.end(),
        "physics_mod");
    const auto game_position = std::find(
        resolution.load_order.begin(),
        resolution.load_order.end(),
        "game");
    EXPECT_LT(physics_position, game_position);
}
