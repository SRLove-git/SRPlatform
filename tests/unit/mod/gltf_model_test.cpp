#include "mod/gltf_model.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace
{

std::string encodeBase64(const std::vector<std::uint8_t>& data)
{
    static constexpr char kTable[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string output;
    for (std::size_t i = 0; i < data.size(); i += 3)
    {
        const std::uint32_t value =
            static_cast<std::uint32_t>(data[i]) << 16 |
            (i + 1 < data.size()
                 ? static_cast<std::uint32_t>(data[i + 1]) << 8
                 : 0U) |
            (i + 2 < data.size()
                 ? static_cast<std::uint32_t>(data[i + 2])
                 : 0U);

        output += kTable[(value >> 18) & 0x3F];
        output += kTable[(value >> 12) & 0x3F];
        output += i + 1 < data.size() ? kTable[(value >> 6) & 0x3F] : '=';
        output += i + 2 < data.size() ? kTable[value & 0x3F] : '=';
    }
    return output;
}

std::vector<std::uint8_t> triangleBuffer()
{
    std::vector<std::uint8_t> data;
    const float positions[9] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f};
    const std::uint16_t indices[3] = {0, 1, 2};

    const auto* position_bytes =
        reinterpret_cast<const std::uint8_t*>(positions);
    data.insert(data.end(), position_bytes, position_bytes + sizeof(positions));

    const auto* index_bytes =
        reinterpret_cast<const std::uint8_t*>(indices);
    data.insert(data.end(), index_bytes, index_bytes + sizeof(indices));
    return data;
}

nlohmann::json triangleGltf(
    const std::string& uri,
    bool with_normals,
    std::uint32_t index_component_type)
{
    const bool short_indices = index_component_type == 5123;
    const std::size_t position_bytes = 36;
    const std::size_t index_bytes = short_indices ? 6 : 12;
    const std::size_t normal_bytes = with_normals ? 36 : 0;

    nlohmann::json buffer_views = nlohmann::json::array();
    std::size_t offset = 0;
    buffer_views.push_back({
        {"buffer", 0},
        {"byteOffset", offset},
        {"byteLength", position_bytes}});
    offset += position_bytes;
    if (with_normals)
    {
        buffer_views.push_back({
            {"buffer", 0},
            {"byteOffset", offset},
            {"byteLength", normal_bytes}});
        offset += normal_bytes;
    }
    buffer_views.push_back({
        {"buffer", 0},
        {"byteOffset", offset},
        {"byteLength", index_bytes}});

    nlohmann::json accessors = nlohmann::json::array();
    accessors.push_back({
        {"bufferView", 0},
        {"componentType", 5126},
        {"count", 3},
        {"type", "VEC3"}});
    std::size_t normal_accessor_index = 0;
    if (with_normals)
    {
        normal_accessor_index = accessors.size();
        accessors.push_back({
            {"bufferView", 1},
            {"componentType", 5126},
            {"count", 3},
            {"type", "VEC3"}});
    }
    const std::size_t indices_view_index = with_normals ? 2 : 1;
    accessors.push_back({
        {"bufferView", indices_view_index},
        {"componentType", index_component_type},
        {"count", 3},
        {"type", "SCALAR"}});

    nlohmann::json attributes = {{"POSITION", 0}};
    if (with_normals)
    {
        attributes["NORMAL"] = normal_accessor_index;
    }

    nlohmann::json gltf;
    gltf["asset"]["version"] = "2.0";
    gltf["scene"] = 0;
    gltf["scenes"] = nlohmann::json::array();
    gltf["scenes"][0]["nodes"] = nlohmann::json::array({0});
    gltf["nodes"] = nlohmann::json::array();
    gltf["nodes"][0]["mesh"] = 0;
    gltf["meshes"] = nlohmann::json::array();
    gltf["meshes"][0]["name"] = "triangle";
    gltf["meshes"][0]["primitives"] = nlohmann::json::array();
    gltf["meshes"][0]["primitives"][0]["attributes"] = attributes;
    gltf["meshes"][0]["primitives"][0]["indices"] =
        with_normals ? 2U : 1U;
    gltf["buffers"] = nlohmann::json::array();
    gltf["buffers"][0]["uri"] = uri;
    gltf["buffers"][0]["byteLength"] =
        position_bytes + normal_bytes + index_bytes;
    gltf["bufferViews"] = buffer_views;
    gltf["accessors"] = accessors;
    return gltf;
}

std::filesystem::path writeTempFile(
    const std::string& name,
    const std::string& content,
    bool binary = false)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_gltf_model_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path path = directory / name;

    std::ofstream stream(
        path,
        binary ? std::ios::binary : std::ios::out);
    stream << content;
    return path;
}

void cleanup()
{
    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_gltf_model_test");
}

}  // namespace

TEST(GltfModel, LoadsEmbeddedBase64Triangle)
{
    const std::vector<std::uint8_t> buffer = triangleBuffer();
    const std::string uri =
        "data:application/octet-stream;base64," + encodeBase64(buffer);
    const std::filesystem::path path = writeTempFile(
        "embedded.gltf",
        triangleGltf(uri, false, 5123).dump());

    std::string error;
    const auto meshes = srp::mod::loadGltfModel(path, error);

    ASSERT_TRUE(meshes.has_value()) << error;
    ASSERT_EQ(meshes->size(), 1U);
    const srp::mod::GltfMesh& mesh = (*meshes)[0];
    EXPECT_EQ(mesh.name, "triangle");
    ASSERT_EQ(mesh.vertices.size(), 3U);
    ASSERT_EQ(mesh.indices.size(), 3U);
    EXPECT_FALSE(mesh.has_normals);
    EXPECT_DOUBLE_EQ(mesh.vertices[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(mesh.vertices[1].position.x, 1.0);
    EXPECT_DOUBLE_EQ(mesh.vertices[2].position.y, 1.0);
    EXPECT_EQ(mesh.indices[0], 0U);
    EXPECT_EQ(mesh.indices[1], 1U);
    EXPECT_EQ(mesh.indices[2], 2U);
    cleanup();
}

TEST(GltfModel, LoadsExternalBinBuffer)
{
    const std::vector<std::uint8_t> buffer = triangleBuffer();
    const std::string bin_content(
        reinterpret_cast<const char*>(buffer.data()),
        buffer.size());
    writeTempFile("mesh.bin", bin_content, true);
    const std::filesystem::path path = writeTempFile(
        "external.gltf",
        triangleGltf("mesh.bin", false, 5123).dump());

    std::string error;
    const auto meshes = srp::mod::loadGltfModel(path, error);

    ASSERT_TRUE(meshes.has_value()) << error;
    ASSERT_EQ(meshes->size(), 1U);
    EXPECT_EQ((*meshes)[0].vertices.size(), 3U);
    cleanup();
}

TEST(GltfModel, LoadsNormals)
{
    const std::vector<std::uint8_t> base = triangleBuffer();
    std::vector<std::uint8_t> buffer(base.begin(), base.begin() + 36);
    const float normals[9] = {
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f};
    const auto* normal_bytes =
        reinterpret_cast<const std::uint8_t*>(normals);
    buffer.insert(buffer.end(), normal_bytes, normal_bytes + sizeof(normals));
    const std::uint16_t indices[3] = {0, 1, 2};
    const auto* index_bytes =
        reinterpret_cast<const std::uint8_t*>(indices);
    buffer.insert(buffer.end(), index_bytes, index_bytes + sizeof(indices));

    const std::string uri =
        "data:application/octet-stream;base64," + encodeBase64(buffer);
    const std::filesystem::path path = writeTempFile(
        "normals.gltf",
        triangleGltf(uri, true, 5123).dump());

    std::string error;
    const auto meshes = srp::mod::loadGltfModel(path, error);

    ASSERT_TRUE(meshes.has_value()) << error;
    const srp::mod::GltfMesh& mesh = (*meshes)[0];
    EXPECT_TRUE(mesh.has_normals);
    ASSERT_EQ(mesh.vertices.size(), 3U);
    EXPECT_DOUBLE_EQ(mesh.vertices[0].normal.z, 1.0);
    EXPECT_DOUBLE_EQ(mesh.vertices[1].normal.z, 1.0);
    EXPECT_DOUBLE_EQ(mesh.vertices[2].normal.z, 1.0);
    cleanup();
}

TEST(GltfModel, AppliesNodeTranslation)
{
    const std::vector<std::uint8_t> buffer = triangleBuffer();
    const std::string uri =
        "data:application/octet-stream;base64," + encodeBase64(buffer);
    nlohmann::json gltf = triangleGltf(uri, false, 5123);
    gltf["nodes"][0]["translation"] = {1.0, 2.0, 3.0};
    const std::filesystem::path path = writeTempFile(
        "translated.gltf",
        gltf.dump());

    std::string error;
    const auto meshes = srp::mod::loadGltfModel(path, error);

    ASSERT_TRUE(meshes.has_value()) << error;
    const srp::mod::GltfMesh& mesh = (*meshes)[0];
    EXPECT_DOUBLE_EQ(mesh.vertices[0].position.x, 1.0);
    EXPECT_DOUBLE_EQ(mesh.vertices[0].position.y, 2.0);
    EXPECT_DOUBLE_EQ(mesh.vertices[0].position.z, 3.0);
    EXPECT_DOUBLE_EQ(mesh.vertices[1].position.x, 2.0);
    cleanup();
}

TEST(GltfModel, GeneratesSequentialIndicesWhenMissing)
{
    const std::vector<std::uint8_t> buffer = triangleBuffer();
    const std::string uri =
        "data:application/octet-stream;base64," + encodeBase64(buffer);
    nlohmann::json gltf = triangleGltf(uri, false, 5123);
    gltf["meshes"][0]["primitives"][0].erase("indices");
    const std::filesystem::path path = writeTempFile(
        "no_indices.gltf",
        gltf.dump());

    std::string error;
    const auto meshes = srp::mod::loadGltfModel(path, error);

    ASSERT_TRUE(meshes.has_value()) << error;
    const srp::mod::GltfMesh& mesh = (*meshes)[0];
    ASSERT_EQ(mesh.indices.size(), 3U);
    EXPECT_EQ(mesh.indices[0], 0U);
    EXPECT_EQ(mesh.indices[1], 1U);
    EXPECT_EQ(mesh.indices[2], 2U);
    cleanup();
}

TEST(GltfModel, SupportsUnsignedIntIndices)
{
    std::vector<std::uint8_t> buffer;
    const float positions[9] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f};
    const auto* position_bytes =
        reinterpret_cast<const std::uint8_t*>(positions);
    buffer.insert(buffer.end(), position_bytes, position_bytes + sizeof(positions));
    const std::uint32_t indices[3] = {0, 1, 2};
    const auto* index_bytes =
        reinterpret_cast<const std::uint8_t*>(indices);
    buffer.insert(buffer.end(), index_bytes, index_bytes + sizeof(indices));
    const std::string uri =
        "data:application/octet-stream;base64," + encodeBase64(buffer);
    const std::filesystem::path path = writeTempFile(
        "u32_indices.gltf",
        triangleGltf(uri, false, 5125).dump());

    std::string error;
    const auto meshes = srp::mod::loadGltfModel(path, error);

    ASSERT_TRUE(meshes.has_value()) << error;
    const srp::mod::GltfMesh& mesh = (*meshes)[0];
    ASSERT_EQ(mesh.indices.size(), 3U);
    EXPECT_EQ(mesh.indices[1], 1U);
    cleanup();
}

TEST(GltfModel, RejectsNonTwoVersionAssets)
{
    nlohmann::json gltf = triangleGltf("nope.bin", false, 5123);
    gltf["asset"]["version"] = "1.0";
    const std::filesystem::path path = writeTempFile(
        "old.gltf",
        gltf.dump());

    std::string error;
    EXPECT_FALSE(srp::mod::loadGltfModel(path, error).has_value());
    EXPECT_FALSE(error.empty());
    cleanup();
}

TEST(GltfModel, RejectsMissingBufferFile)
{
    const std::filesystem::path path = writeTempFile(
        "missing_buffer.gltf",
        triangleGltf("does_not_exist.bin", false, 5123).dump());

    std::string error;
    EXPECT_FALSE(srp::mod::loadGltfModel(path, error).has_value());
    EXPECT_FALSE(error.empty());
    cleanup();
}

TEST(GltfModel, RejectsInvalidBase64)
{
    const std::filesystem::path path = writeTempFile(
        "bad_base64.gltf",
        triangleGltf(
            "data:application/octet-stream;base64,!!!not-base64!!!",
            false,
            5123).dump());

    std::string error;
    EXPECT_FALSE(srp::mod::loadGltfModel(path, error).has_value());
    EXPECT_FALSE(error.empty());
    cleanup();
}

TEST(GltfModel, RejectsUnsupportedPositionType)
{
    const std::vector<std::uint8_t> buffer = triangleBuffer();
    const std::string uri =
        "data:application/octet-stream;base64," + encodeBase64(buffer);
    nlohmann::json gltf = triangleGltf(uri, false, 5123);
    gltf["accessors"][0]["componentType"] = 5123;
    const std::filesystem::path path = writeTempFile(
        "bad_position.gltf",
        gltf.dump());

    std::string error;
    EXPECT_FALSE(srp::mod::loadGltfModel(path, error).has_value());
    EXPECT_FALSE(error.empty());
    cleanup();
}

TEST(GltfModel, RejectsMissingPositionAttribute)
{
    const std::vector<std::uint8_t> buffer = triangleBuffer();
    const std::string uri =
        "data:application/octet-stream;base64," + encodeBase64(buffer);
    nlohmann::json gltf = triangleGltf(uri, false, 5123);
    gltf["meshes"][0]["primitives"][0]["attributes"].erase("POSITION");
    const std::filesystem::path path = writeTempFile(
        "no_position.gltf",
        gltf.dump());

    std::string error;
    EXPECT_FALSE(srp::mod::loadGltfModel(path, error).has_value());
    EXPECT_FALSE(error.empty());
    cleanup();
}
