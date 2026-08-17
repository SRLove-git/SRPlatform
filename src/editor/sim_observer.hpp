#pragma once

#include "bridge/car_entity.hpp"
#include "editor/time_series.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace srp::physics
{
class PhysicsWorld;
}

namespace srp::editor
{

// Collects named time series from the running simulation. Each channel is a
// fixed-capacity TimeSeries keyed by a stable name, e.g.
// "battery_voltage_v" or "sensor_distance_front_m".
class SimObserver
{
public:
    explicit SimObserver(std::size_t samples_per_channel = 1800);

    void setSimTime(double time);
    double simTime() const;

    void record(const std::string& channel, double value);
    void recordSensor(const std::string& channel, double value);

    // Integrates value_per_second over dt and records the running total in
    // the channel (used for energy consumption).
    void accumulate(
        const std::string& channel,
        double value_per_second,
        double dt);

    TimeSeries& channel(const std::string& name);
    const TimeSeries& channel(const std::string& name) const;
    bool hasChannel(const std::string& name) const;

    const std::vector<std::string>& channelNames() const;

    void clear();

private:
    TimeSeries& ensureChannel(const std::string& name);

    std::unordered_map<std::string, TimeSeries> channels_;
    std::vector<std::string> channel_names_;
    double sim_time_{0.0};
    std::unordered_map<std::string, double> accumulators_;
    std::size_t capacity_;
};

// Samples the example car's electrical, mechanical, contact, and energy
// telemetry into the observer. dt is the fixed simulation step.
void sampleCarTelemetry(
    const bridge::CarEntity& car,
    const physics::PhysicsWorld& world,
    double dt,
    SimObserver& observer);

}  // namespace srp::editor
