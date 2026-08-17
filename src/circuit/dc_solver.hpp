#pragma once

#include "circuit/circuit_model.hpp"

#include <optional>
#include <vector>

namespace srp::circuit
{

struct DcNodeVoltage
{
    NodeId node{kInvalidNodeId};
    Voltage voltage{0.0};
};

struct DcComponentCurrent
{
    ComponentId component{kInvalidComponentId};
    Current current{0.0};
};

struct DcAnalysisResult
{
    std::vector<DcNodeVoltage> node_voltages;
    std::vector<DcComponentCurrent> component_currents;
};

std::optional<Voltage> nodeVoltage(const DcAnalysisResult& result, NodeId node);
std::optional<Current> componentCurrent(
    const DcAnalysisResult& result,
    ComponentId component);

// Solves the operating point of a linear DC circuit.
//
// Supported component types are resistors, independent voltage sources, and
// independent current sources. Node voltages are relative to kGroundNodeId.
// For every two-port component, branch current is referenced from port[0] to
// port[1]. A voltage source contributes one extra unknown: its current from
// its positive port to its negative port.
std::optional<DcAnalysisResult> solveDc(const CircuitModel& circuit);

}  // namespace srp::circuit
