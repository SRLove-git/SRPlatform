#pragma once

#include "circuit/circuit_types.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace srp::circuit
{

class CircuitModel;

struct TransientSettings
{
    double time_step{1e-4};
    double end_time{1.0};
};

struct TransientResult
{
    std::vector<double> times;
    std::vector<NodeId> nodes;
    std::vector<ComponentId> components;
    std::vector<std::vector<Voltage>> node_voltages;
    std::vector<std::vector<Current>> component_currents;
};

std::optional<Voltage> nodeVoltageAt(
    const TransientResult& result,
    NodeId node,
    std::size_t sample_index);

std::optional<Current> componentCurrentAt(
    const TransientResult& result,
    ComponentId component,
    std::size_t sample_index);

// Simulates a linear RC/RL/RLC circuit with fixed time steps.
//
// Capacitors use implicit Euler integration and respect their initial voltage.
// Inductors use implicit Euler integration and respect their initial current.
// Independent sources are currently held constant during the simulation.
std::optional<TransientResult> simulateTransient(
    const CircuitModel& circuit,
    const TransientSettings& settings);

}  // namespace srp::circuit
