#include "mod/mod_package.hpp"

#include <filesystem>

namespace srp::mod
{

std::filesystem::path ModPackage::entryPath() const
{
    return root / manifest.entry;
}

std::optional<ModPackage> loadModPackage(
    const std::filesystem::path& directory,
    std::string& error)
{
    error.clear();

    const std::filesystem::path manifest_path = directory / "mod.json";
    const std::optional<ModManifest> manifest =
        loadManifest(manifest_path, error);
    if (!manifest.has_value())
    {
        return std::nullopt;
    }

    ModPackage package;
    package.manifest = *manifest;
    package.root = directory;

    std::error_code ec;
    if (!std::filesystem::is_regular_file(package.entryPath(), ec))
    {
        error = "mod entry script does not exist: " +
                package.entryPath().string();
        return std::nullopt;
    }

    return package;
}

}  // namespace srp::mod
