#include "mod/gltf_model.hpp"

#include <fstream>
#include <sstream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <nlohmann/json.hpp>

namespace srp::mod
{
namespace
{

constexpr std::uint32_t kComponentByte = 5120;
constexpr std::uint32_t kComponentUByte = 5121;
constexpr std::uint32_t kComponentShort = 5122;
constexpr std::uint32_t kComponentUShort = 5123;
constexpr std::uint32_t kComponentUInt = 5125;
constexpr std::uint32_t kComponentFloat = 5126;

std::uint16_t readU16LE(const std::uint8_t* bytes)
{
    return static_cast<std::uint16_t>(bytes[0]) |
           static_cast<std::uint16_t>(bytes[1]) << 8;
}

std::uint32_t readU32LE(const std::uint8_t* bytes)
{
    return static_cast<std::uint32_t>(bytes[0]) |
           static_cast<std::uint32_t>(bytes[1]) << 8 |
           static_cast<std::uint32_t>(bytes[2]) << 16 |
           static_cast<std::uint32_t>(bytes[3]) << 24;
}

float readFloatLE(const std::uint8_t* bytes)
{
    const std::uint32_t bits = readU32LE(bytes);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::optional<std::vector<std::uint8_t>> decodeBase64(
    const std::string& input)
{
    static constexpr int kDecodeTable[256] = {
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63,
        52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -1, -1, -1,
        -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
        15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1,
        -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
        41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

    std::vector<std::uint8_t> output;
    output.reserve(input.size() * 3 / 4);

    int accumulator = 0;
    int accumulator_bits = 0;
    for (const unsigned char character : input)
    {
        if (character == '=' || character == '\n' || character == '\r')
        {
            continue;
        }

        const int value = kDecodeTable[character];
        if (value < 0)
        {
            return std::nullopt;
        }

        accumulator = (accumulator << 6) | value;
        accumulator_bits += 6;
        if (accumulator_bits >= 8)
        {
            accumulator_bits -= 8;
            output.push_back(
                static_cast<std::uint8_t>((accumulator >> accumulator_bits) & 0xFF));
        }
    }

    return output;
}

std::optional<std::vector<std::uint8_t>> loadBuffer(
    const nlohmann::json& buffer,
    const std::filesystem::path& base_directory,
    std::string& error)
{
    const auto uri_it = buffer.find("uri");
    if (uri_it == buffer.end() || !uri_it->is_string())
    {
        error = "glTF buffer requires a 'uri' (GLB containers are not supported)";
        return std::nullopt;
    }

    const std::string uri = uri_it->get<std::string>();
    std::vector<std::uint8_t> data;

    if (uri.rfind("data:", 0) == 0)
    {
        const std::string marker = ";base64,";
        const std::size_t marker_position = uri.find(marker);
        if (marker_position == std::string::npos)
        {
            error = "only base64 data URIs are supported";
            return std::nullopt;
        }

        const auto decoded = decodeBase64(
            uri.substr(marker_position + marker.size()));
        if (!decoded.has_value())
        {
            error = "invalid base64 data in buffer URI";
            return std::nullopt;
        }
        data = *decoded;
    }
    else
    {
        std::ifstream stream(base_directory / uri, std::ios::binary);
        if (!stream.is_open())
        {
            error = "cannot open glTF buffer file: " + uri;
            return std::nullopt;
        }

        stream.seekg(0, std::ios::end);
        const std::streamsize size = stream.tellg();
        stream.seekg(0, std::ios::beg);
        data.resize(static_cast<std::size_t>(size));
        if (size > 0)
        {
            stream.read(
                reinterpret_cast<char*>(data.data()),
                size);
        }
    }

    const auto length_it = buffer.find("byteLength");
    if (length_it != buffer.end() && length_it->is_number_unsigned())
    {
        const std::size_t expected = length_it->get<std::size_t>();
        if (data.size() < expected)
        {
            error = "glTF buffer is shorter than declared byteLength";
            return std::nullopt;
        }
        data.resize(expected);
    }

    return data;
}

struct Accessor
{
    std::uint32_t component_type{0};
    std::size_t component_count{0};
    std::size_t count{0};
    std::optional<std::size_t> buffer_view;
    std::size_t byte_offset{0};
};

std::optional<Accessor> parseAccessor(
    const nlohmann::json& accessors,
    std::size_t index,
    std::string& error)
{
    if (index >= accessors.size() || !accessors[index].is_object())
    {
        error = "glTF accessor index out of range";
        return std::nullopt;
    }

    const nlohmann::json& accessor = accessors[index];
    const auto component_it = accessor.find("componentType");
    const auto count_it = accessor.find("count");
    const auto type_it = accessor.find("type");
    if (component_it == accessor.end() ||
        count_it == accessor.end() ||
        type_it == accessor.end())
    {
        error = "glTF accessor requires componentType, count and type";
        return std::nullopt;
    }

    Accessor result;
    result.component_type = component_it->get<std::uint32_t>();
    result.count = count_it->get<std::size_t>();

    const std::string type = type_it->get<std::string>();
    if (type == "SCALAR")
    {
        result.component_count = 1;
    }
    else if (type == "VEC2")
    {
        result.component_count = 2;
    }
    else if (type == "VEC3")
    {
        result.component_count = 3;
    }
    else if (type == "VEC4")
    {
        result.component_count = 4;
    }
    else
    {
        error = "unsupported glTF accessor type: " + type;
        return std::nullopt;
    }

    if (const auto view_it = accessor.find("bufferView");
        view_it != accessor.end())
    {
        result.buffer_view = view_it->get<std::size_t>();
    }
    if (const auto offset_it = accessor.find("byteOffset");
        offset_it != accessor.end())
    {
        result.byte_offset = offset_it->get<std::size_t>();
    }

    return result;
}

std::size_t componentSize(std::uint32_t component_type)
{
    switch (component_type)
    {
    case kComponentByte:
    case kComponentUByte:
        return 1;
    case kComponentShort:
    case kComponentUShort:
        return 2;
    case kComponentUInt:
    case kComponentFloat:
        return 4;
    default:
        return 0;
    }
}

struct BufferView
{
    std::size_t buffer{0};
    std::size_t byte_offset{0};
    std::size_t byte_length{0};
    std::optional<std::size_t> byte_stride;
};

std::optional<BufferView> parseBufferView(
    const nlohmann::json& buffer_views,
    std::size_t index,
    std::string& error)
{
    if (index >= buffer_views.size() || !buffer_views[index].is_object())
    {
        error = "glTF bufferView index out of range";
        return std::nullopt;
    }

    const nlohmann::json& view = buffer_views[index];
    BufferView result;
    result.buffer = view.at("buffer").get<std::size_t>();
    result.byte_length = view.at("byteLength").get<std::size_t>();
    if (const auto offset_it = view.find("byteOffset");
        offset_it != view.end())
    {
        result.byte_offset = offset_it->get<std::size_t>();
    }
    if (const auto stride_it = view.find("byteStride");
        stride_it != view.end())
    {
        result.byte_stride = stride_it->get<std::size_t>();
    }
    return result;
}

math::Vec3 readVec3Float(
    const std::vector<std::uint8_t>& buffer,
    std::size_t offset)
{
    return math::Vec3(
        readFloatLE(buffer.data() + offset),
        readFloatLE(buffer.data() + offset + 4),
        readFloatLE(buffer.data() + offset + 8));
}

std::uint32_t readIndex(
    const std::vector<std::uint8_t>& buffer,
    std::size_t offset,
    std::uint32_t component_type)
{
    if (component_type == kComponentUShort)
    {
        return readU16LE(buffer.data() + offset);
    }
    return readU32LE(buffer.data() + offset);
}

glm::dmat4 nodeTransform(
    const nlohmann::json& node,
    std::string& error)
{
    if (const auto matrix_it = node.find("matrix");
        matrix_it != node.end())
    {
        if (!matrix_it->is_array() || matrix_it->size() != 16)
        {
            error = "glTF node matrix must have 16 numbers";
            return glm::dmat4(1.0);
        }

        double values[16];
        for (std::size_t i = 0; i < 16; ++i)
        {
            if (!(*matrix_it)[i].is_number())
            {
                error = "glTF node matrix entries must be numbers";
                return glm::dmat4(1.0);
            }
            values[i] = (*matrix_it)[i].get<double>();
        }
        return glm::dmat4(
            values[0], values[1], values[2], values[3],
            values[4], values[5], values[6], values[7],
            values[8], values[9], values[10], values[11],
            values[12], values[13], values[14], values[15]);
    }

    glm::dvec3 translation(0.0);
    glm::dquat rotation(1.0, 0.0, 0.0, 0.0);
    glm::dvec3 scale(1.0);

    if (const auto translation_it = node.find("translation");
        translation_it != node.end() &&
        translation_it->is_array() &&
        translation_it->size() == 3)
    {
        translation = glm::dvec3(
            (*translation_it)[0].get<double>(),
            (*translation_it)[1].get<double>(),
            (*translation_it)[2].get<double>());
    }

    if (const auto rotation_it = node.find("rotation");
        rotation_it != node.end() &&
        rotation_it->is_array() &&
        rotation_it->size() == 4)
    {
        rotation = glm::dquat(
            (*rotation_it)[3].get<double>(),
            (*rotation_it)[0].get<double>(),
            (*rotation_it)[1].get<double>(),
            (*rotation_it)[2].get<double>());
    }

    if (const auto scale_it = node.find("scale");
        scale_it != node.end() &&
        scale_it->is_array() &&
        scale_it->size() == 3)
    {
        scale = glm::dvec3(
            (*scale_it)[0].get<double>(),
            (*scale_it)[1].get<double>(),
            (*scale_it)[2].get<double>());
    }

    const glm::dmat4 translation_matrix = glm::translate(
        glm::dmat4(1.0), translation);
    const glm::dmat4 rotation_matrix = glm::mat4_cast(rotation);
    const glm::dmat4 scale_matrix = glm::scale(
        glm::dmat4(1.0), scale);
    return translation_matrix * rotation_matrix * scale_matrix;
}

std::optional<GltfMesh> readPrimitive(
    const nlohmann::json& primitive,
    const std::string& mesh_name,
    const nlohmann::json& accessors,
    const nlohmann::json& buffer_views,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    const glm::dmat4& transform,
    std::string& error)
{
    if (!primitive.is_object())
    {
        error = "glTF primitive must be an object";
        return std::nullopt;
    }

    const auto attributes_it = primitive.find("attributes");
    if (attributes_it == primitive.end() || !attributes_it->is_object())
    {
        error = "glTF primitive requires an attributes object";
        return std::nullopt;
    }

    const auto position_it = attributes_it->find("POSITION");
    if (position_it == attributes_it->end())
    {
        error = "glTF primitive requires a POSITION attribute";
        return std::nullopt;
    }

    const std::optional<Accessor> position_accessor =
        parseAccessor(accessors, position_it->get<std::size_t>(), error);
    if (!position_accessor.has_value())
    {
        return std::nullopt;
    }
    if (position_accessor->component_type != kComponentFloat ||
        position_accessor->component_count != 3 ||
        !position_accessor->buffer_view.has_value())
    {
        error = "POSITION attribute must be FLOAT VEC3 backed by a bufferView";
        return std::nullopt;
    }

    const std::optional<BufferView> position_view = parseBufferView(
        buffer_views, *position_accessor->buffer_view, error);
    if (!position_view.has_value())
    {
        return std::nullopt;
    }
    if (position_view->buffer >= buffers.size())
    {
        error = "glTF bufferView references an out-of-range buffer";
        return std::nullopt;
    }
    const std::size_t position_stride =
        position_view->byte_stride.value_or(12);

    std::optional<Accessor> normal_accessor;
    std::optional<BufferView> normal_view;
    std::size_t normal_stride = 12;
    if (const auto normal_it = attributes_it->find("NORMAL");
        normal_it != attributes_it->end())
    {
        normal_accessor =
            parseAccessor(accessors, normal_it->get<std::size_t>(), error);
        if (!normal_accessor.has_value())
        {
            return std::nullopt;
        }
        if (normal_accessor->component_type != kComponentFloat ||
            normal_accessor->component_count != 3 ||
            !normal_accessor->buffer_view.has_value())
        {
            error = "NORMAL attribute must be FLOAT VEC3 backed by a bufferView";
            return std::nullopt;
        }
        normal_view = parseBufferView(
            buffer_views, *normal_accessor->buffer_view, error);
        if (!normal_view.has_value())
        {
            return std::nullopt;
        }
        normal_stride = normal_view->byte_stride.value_or(12);
    }

    GltfMesh mesh;
    mesh.name = mesh_name;
    mesh.has_normals = normal_accessor.has_value();
    mesh.vertices.reserve(position_accessor->count);

    const glm::dmat3 rotation = glm::dmat3(transform);
    const std::vector<std::uint8_t>& position_buffer =
        buffers[position_view->buffer];

    for (std::size_t i = 0; i < position_accessor->count; ++i)
    {
        const std::size_t offset =
            position_view->byte_offset +
            position_accessor->byte_offset +
            i * position_stride;
        const glm::dvec4 world_position =
            transform * glm::dvec4(
                readVec3Float(position_buffer, offset), 1.0);

        GltfVertex vertex;
        vertex.position = math::Vec3(
            world_position.x, world_position.y, world_position.z);

        if (normal_accessor.has_value())
        {
            const std::vector<std::uint8_t>& normal_buffer =
                buffers[normal_view->buffer];
            const std::size_t normal_offset =
                normal_view->byte_offset +
                normal_accessor->byte_offset +
                i * normal_stride;
            const glm::dvec3 world_normal = glm::normalize(
                rotation * glm::dvec3(
                    readVec3Float(normal_buffer, normal_offset)));
            vertex.normal = math::Vec3(
                world_normal.x, world_normal.y, world_normal.z);
        }

        mesh.vertices.push_back(vertex);
    }

    if (const auto indices_it = primitive.find("indices");
        indices_it != primitive.end())
    {
        const std::optional<Accessor> indices_accessor =
            parseAccessor(accessors, indices_it->get<std::size_t>(), error);
        if (!indices_accessor.has_value())
        {
            return std::nullopt;
        }
        if (indices_accessor->component_count != 1 ||
            (indices_accessor->component_type != kComponentUShort &&
             indices_accessor->component_type != kComponentUInt) ||
            !indices_accessor->buffer_view.has_value())
        {
            error = "indices accessor must be SCALAR UNSIGNED_SHORT or UNSIGNED_INT";
            return std::nullopt;
        }

        const std::optional<BufferView> indices_view = parseBufferView(
            buffer_views, *indices_accessor->buffer_view, error);
        if (!indices_view.has_value())
        {
            return std::nullopt;
        }
        if (indices_view->buffer >= buffers.size())
        {
            error = "glTF bufferView references an out-of-range buffer";
            return std::nullopt;
        }
        const std::size_t index_stride =
            indices_view->byte_stride.value_or(
                componentSize(indices_accessor->component_type));
        const std::vector<std::uint8_t>& indices_buffer =
            buffers[indices_view->buffer];

        mesh.indices.reserve(indices_accessor->count);
        for (std::size_t i = 0; i < indices_accessor->count; ++i)
        {
            const std::size_t offset =
                indices_view->byte_offset +
                indices_accessor->byte_offset +
                i * index_stride;
            mesh.indices.push_back(readIndex(
                indices_buffer,
                offset,
                indices_accessor->component_type));
        }
    }
    else
    {
        mesh.indices.resize(mesh.vertices.size());
        for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
        {
            mesh.indices[i] = static_cast<std::uint32_t>(i);
        }
    }

    return mesh;
}

void collectNodeMeshes(
    const nlohmann::json& nodes,
    std::size_t node_index,
    const glm::dmat4& parent_transform,
    const nlohmann::json& meshes,
    const nlohmann::json& accessors,
    const nlohmann::json& buffer_views,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    std::vector<GltfMesh>& output,
    std::string& error)
{
    if (node_index >= nodes.size() || !nodes[node_index].is_object())
    {
        error = "glTF node index out of range";
        return;
    }

    const nlohmann::json& node = nodes[node_index];
    const glm::dmat4 local_transform = nodeTransform(node, error);
    const glm::dmat4 world_transform = parent_transform * local_transform;

    if (const auto mesh_it = node.find("mesh");
        mesh_it != node.end() &&
        mesh_it->is_number_unsigned())
    {
        const std::size_t mesh_index = mesh_it->get<std::size_t>();
        if (mesh_index >= meshes.size() || !meshes[mesh_index].is_object())
        {
            error = "glTF mesh index out of range";
            return;
        }

        const nlohmann::json& mesh = meshes[mesh_index];
        std::string mesh_name = "mesh";
        if (const auto name_it = mesh.find("name");
            name_it != mesh.end() && name_it->is_string())
        {
            mesh_name = name_it->get<std::string>();
        }

        const auto primitives_it = mesh.find("primitives");
        if (primitives_it == mesh.end() || !primitives_it->is_array())
        {
            error = "glTF mesh requires a primitives array";
            return;
        }

        for (std::size_t i = 0; i < primitives_it->size(); ++i)
        {
            std::string primitive_name = mesh_name;
            if (primitives_it->size() > 1)
            {
                primitive_name += "#" + std::to_string(i);
            }

            const std::optional<GltfMesh> primitive_mesh = readPrimitive(
                (*primitives_it)[i],
                primitive_name,
                accessors,
                buffer_views,
                buffers,
                world_transform,
                error);
            if (primitive_mesh.has_value())
            {
                output.push_back(*primitive_mesh);
            }
            else if (!error.empty())
            {
                return;
            }
        }
    }

    if (const auto children_it = node.find("children");
        children_it != node.end() && children_it->is_array())
    {
        for (const nlohmann::json& child : *children_it)
        {
            if (!child.is_number_unsigned())
            {
                error = "glTF node children entries must be integers";
                return;
            }
            collectNodeMeshes(
                nodes,
                child.get<std::size_t>(),
                world_transform,
                meshes,
                accessors,
                buffer_views,
                buffers,
                output,
                error);
        }
    }
}

}  // namespace

std::optional<std::vector<GltfMesh>> loadGltfModel(
    const std::filesystem::path& path,
    std::string& error)
{
    error.clear();

    std::ifstream stream(path);
    if (!stream.is_open())
    {
        error = "cannot open glTF file: " + path.string();
        return std::nullopt;
    }

    nlohmann::json root;
    try
    {
        stream >> root;
    }
    catch (const nlohmann::json::parse_error& parse_error)
    {
        error = std::string("invalid JSON in glTF file: ") + parse_error.what();
        return std::nullopt;
    }

    if (!root.is_object())
    {
        error = "glTF root must be a JSON object";
        return std::nullopt;
    }

    const auto asset_it = root.find("asset");
    if (asset_it == root.end() || !asset_it->is_object())
    {
        error = "glTF requires an asset object";
        return std::nullopt;
    }
    const auto version_it = asset_it->find("version");
    if (version_it == asset_it->end() ||
        !version_it->is_string() ||
        version_it->get<std::string>().substr(0, 3) != "2.0")
    {
        error = "only glTF 2.0 assets are supported";
        return std::nullopt;
    }

    const auto buffers_it = root.find("buffers");
    if (buffers_it == root.end() || !buffers_it->is_array())
    {
        error = "glTF requires a buffers array";
        return std::nullopt;
    }

    std::vector<std::vector<std::uint8_t>> buffers;
    const std::filesystem::path base_directory = path.parent_path();
    for (const nlohmann::json& buffer : *buffers_it)
    {
        const auto data = loadBuffer(buffer, base_directory, error);
        if (!data.has_value())
        {
            return std::nullopt;
        }
        buffers.push_back(*data);
    }

    const nlohmann::json empty_array = nlohmann::json::array();
    const nlohmann::json& buffer_views =
        root.value("bufferViews", empty_array);
    const nlohmann::json& accessors =
        root.value("accessors", empty_array);
    const nlohmann::json& meshes =
        root.value("meshes", empty_array);
    const nlohmann::json& nodes =
        root.value("nodes", empty_array);

    std::vector<GltfMesh> output;
    const auto scenes_it = root.find("scenes");
    if (scenes_it != root.end() && scenes_it->is_array() &&
        !scenes_it->empty())
    {
        const nlohmann::json& scene = (*scenes_it)[0];
        const auto scene_nodes_it = scene.find("nodes");
        if (scene_nodes_it != scene.end() && scene_nodes_it->is_array())
        {
            for (const nlohmann::json& node_index : *scene_nodes_it)
            {
                if (!node_index.is_number_unsigned())
                {
                    error = "glTF scene node entries must be integers";
                    return std::nullopt;
                }
                collectNodeMeshes(
                    nodes,
                    node_index.get<std::size_t>(),
                    glm::dmat4(1.0),
                    meshes,
                    accessors,
                    buffer_views,
                    buffers,
                    output,
                    error);
                if (!error.empty())
                {
                    return std::nullopt;
                }
            }
        }
    }

    return output;
}

}  // namespace srp::mod
