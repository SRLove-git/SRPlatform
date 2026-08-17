#include "editor/simulation_recorder.hpp"

#include <fstream>

namespace srp::editor
{

void SimulationRecorder::begin()
{
    snapshots_.clear();
    recording_ = true;
}

void SimulationRecorder::end()
{
    recording_ = false;
}

bool SimulationRecorder::recording() const
{
    return recording_;
}

void SimulationRecorder::record(
    double time_s,
    std::vector<BodySnapshot> bodies)
{
    if (!recording_)
    {
        return;
    }
    snapshots_.push_back(SimSnapshot{time_s, std::move(bodies)});
}

std::size_t SimulationRecorder::size() const
{
    return snapshots_.size();
}

const std::vector<SimSnapshot>& SimulationRecorder::snapshots() const
{
    return snapshots_;
}

void SimulationRecorder::clear()
{
    snapshots_.clear();
    recording_ = false;
}

nlohmann::json SimulationRecorder::toJson(
    const std::vector<SimSnapshot>& snapshots)
{
    nlohmann::json json;
    json["version"] = 1;

    nlohmann::json snapshot_array = nlohmann::json::array();
    for (const SimSnapshot& snapshot : snapshots)
    {
        nlohmann::json snapshot_json;
        snapshot_json["t"] = snapshot.time_s;

        nlohmann::json body_array = nlohmann::json::array();
        for (const BodySnapshot& body : snapshot.bodies)
        {
            nlohmann::json body_json;
            body_json["name"] = body.name;
            body_json["kind"] = body.kind;
            body_json["position"] = {
                body.position.x,
                body.position.y,
                body.position.z};
            body_json["orientation"] = {
                body.orientation.w,
                body.orientation.x,
                body.orientation.y,
                body.orientation.z};
            body_array.push_back(std::move(body_json));
        }
        snapshot_json["bodies"] = std::move(body_array);
        snapshot_array.push_back(std::move(snapshot_json));
    }

    json["snapshots"] = std::move(snapshot_array);
    return json;
}

bool SimulationRecorder::fromJson(
    const nlohmann::json& json,
    std::vector<SimSnapshot>& snapshots,
    std::string& error)
{
    snapshots.clear();
    error.clear();

    if (!json.is_object())
    {
        error = "recording must be a JSON object";
        return false;
    }

    const auto snapshots_it = json.find("snapshots");
    if (snapshots_it == json.end() || !snapshots_it->is_array())
    {
        error = "recording requires a 'snapshots' array";
        return false;
    }

    snapshots.reserve(snapshots_it->size());
    for (const nlohmann::json& entry : *snapshots_it)
    {
        if (!entry.is_object())
        {
            error = "recording snapshot entries must be objects";
            return false;
        }

        SimSnapshot snapshot;
        snapshot.time_s = entry.value("t", 0.0);

        const auto bodies_it = entry.find("bodies");
        if (bodies_it == entry.end() || !bodies_it->is_array())
        {
            error = "recording snapshot requires a 'bodies' array";
            return false;
        }

        for (const nlohmann::json& body_entry : *bodies_it)
        {
            BodySnapshot body;
            body.name = body_entry.value("name", std::string{});
            body.kind = body_entry.value("kind", std::string{});

            if (body_entry.contains("position") &&
                body_entry["position"].is_array() &&
                body_entry["position"].size() == 3)
            {
                body.position = srp::math::Vec3(
                    body_entry["position"][0].get<double>(),
                    body_entry["position"][1].get<double>(),
                    body_entry["position"][2].get<double>());
            }
            if (body_entry.contains("orientation") &&
                body_entry["orientation"].is_array() &&
                body_entry["orientation"].size() == 4)
            {
                body.orientation = srp::math::Quat(
                    body_entry["orientation"][0].get<double>(),
                    body_entry["orientation"][1].get<double>(),
                    body_entry["orientation"][2].get<double>(),
                    body_entry["orientation"][3].get<double>());
            }
            snapshot.bodies.push_back(std::move(body));
        }

        snapshots.push_back(std::move(snapshot));
    }
    return true;
}

bool SimulationRecorder::save(
    const std::filesystem::path& path,
    std::string& error) const
{
    error.clear();
    std::ofstream stream(path);
    if (!stream.is_open())
    {
        error = "cannot write recording file: " + path.string();
        return false;
    }

    stream << toJson(snapshots_).dump(2);
    return stream.good();
}

bool SimulationRecorder::load(
    const std::filesystem::path& path,
    std::string& error)
{
    error.clear();
    std::ifstream stream(path);
    if (!stream.is_open())
    {
        error = "cannot open recording file: " + path.string();
        return false;
    }

    try
    {
        const nlohmann::json json = nlohmann::json::parse(stream);
        if (!fromJson(json, snapshots_, error))
        {
            return false;
        }
    }
    catch (const std::exception& exception)
    {
        error = "invalid recording JSON: " + std::string(exception.what());
        return false;
    }

    recording_ = false;
    return true;
}

}  // namespace srp::editor
