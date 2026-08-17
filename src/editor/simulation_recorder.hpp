#pragma once

#include "core/math/types.hpp"

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace srp::editor
{

struct BodySnapshot
{
    std::string name;
    std::string kind;
    srp::math::Vec3 position{0.0};
    srp::math::Quat orientation{1.0, 0.0, 0.0, 0.0};
};

struct SimSnapshot
{
    double time_s{0.0};
    std::vector<BodySnapshot> bodies;
};

// Records fixed-step simulation snapshots (body transforms at each step) and
// serializes them to the JSON format documented in docs/recording-format.md.
class SimulationRecorder
{
public:
    void begin();
    void end();
    bool recording() const;

    void record(double time_s, std::vector<BodySnapshot> bodies);

    std::size_t size() const;
    const std::vector<SimSnapshot>& snapshots() const;
    void clear();

    bool save(
        const std::filesystem::path& path,
        std::string& error) const;
    bool load(
        const std::filesystem::path& path,
        std::string& error);

    static nlohmann::json toJson(const std::vector<SimSnapshot>& snapshots);
    static bool fromJson(
        const nlohmann::json& json,
        std::vector<SimSnapshot>& snapshots,
        std::string& error);

private:
    std::vector<SimSnapshot> snapshots_;
    bool recording_{false};
};

}  // namespace srp::editor
