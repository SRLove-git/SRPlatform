#pragma once

#include "core/math/types.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace srp::mod
{

struct GltfVertex
{
    math::Vec3 position{0.0};
    math::Vec3 normal{0.0};
};

// A single drawable mesh extracted from a glTF 2.0 asset. All vertices are
// already in the world space of the loaded scene (node transforms applied).
struct GltfMesh
{
    std::string name;
    std::vector<GltfVertex> vertices;
    std::vector<std::uint32_t> indices;
    bool has_normals{false};
};

// Loads a glTF 2.0 .gltf file and returns its drawable meshes. Supports
// embedded base64 buffers and external .bin buffers, FLOAT VEC3 position and
// normal attributes, UNSIGNED_SHORT/UNSIGNED_INT indices, and node
// matrix/TRS transforms. On failure returns nullopt and fills error.
std::optional<std::vector<GltfMesh>> loadGltfModel(
    const std::filesystem::path& path,
    std::string& error);

}  // namespace srp::mod
