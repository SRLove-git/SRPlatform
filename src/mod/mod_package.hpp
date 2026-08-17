#pragma once

#include "mod/mod_manifest.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace srp::mod
{

// A loaded mod directory: its manifest plus the resolved root path. All
// resource paths in the manifest are relative to root.
struct ModPackage
{
    ModManifest manifest;
    std::filesystem::path root;

    std::filesystem::path entryPath() const;
};

// Loads the mod.json inside directory and verifies that the entry script
// exists. On failure returns nullopt and fills error.
std::optional<ModPackage> loadModPackage(
    const std::filesystem::path& directory,
    std::string& error);

}  // namespace srp::mod
