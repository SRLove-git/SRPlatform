#pragma once

#include "circuit/circuit_model.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace srp::circuit
{

struct DigitalSettings
{
    double time_step{1e-3};
    double end_time{1.0};
};

struct DigitalResult
{
    std::vector<double> times;
    std::vector<NodeId> nodes;
    std::vector<std::vector<bool>> node_values;
};

std::optional<bool> nodeValueAt(
    const DigitalResult& result,
    NodeId node,
    std::size_t sample_index);

// Simulates a digital-only subcircuit. Digital source components can be
// constant or produce a square wave, logic gates are combinational, and
// D flip-flops update on the rising edge of their clock input.
std::optional<DigitalResult> simulateDigital(
    const CircuitModel& circuit,
    const DigitalSettings& settings);

}  // namespace srp::circuit
