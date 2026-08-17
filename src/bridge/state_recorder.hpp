#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace srp::bridge
{

struct CarStateSample
{
    double time_s{0.0};
    double battery_voltage_v{0.0};
    double motor_current_a{0.0};
    double motor_angular_velocity_rad_s{0.0};
    double chassis_position_x_m{0.0};
};

// Append-only recorder for the observable state of a car simulation.
class StateRecorder
{
public:
    void record(const CarStateSample& sample);
    void clear();

    std::size_t size() const;
    const std::vector<CarStateSample>& samples() const;
    std::optional<CarStateSample> latest() const;

private:
    std::vector<CarStateSample> samples_;
};

}  // namespace srp::bridge
