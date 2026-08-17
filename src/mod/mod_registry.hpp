#pragma once

#include "mod/mod_manifest.hpp"

#include <string>
#include <vector>

namespace srp::mod
{

// Result of validating a collection of mods: duplicate ids, missing
// dependencies, and dependency cycles are reported as diagnostics.
struct DependencyResolution
{
    bool ok{false};
    std::vector<std::string> errors;

    // Valid dependency order (dependencies before dependents) when ok.
    std::vector<std::string> load_order;
};

// Checks the manifests for duplicate ids and missing dependencies, and
// detects dependency cycles. When valid, load_order lists the mod ids in an
// order where every mod appears after its dependencies.
DependencyResolution resolveModDependencies(
    const std::vector<ModManifest>& manifests);

}  // namespace srp::mod
