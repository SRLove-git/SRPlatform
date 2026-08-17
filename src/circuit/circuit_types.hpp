#pragma once

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace srp::circuit
{

using NodeId = std::uint32_t;
using ComponentId = std::uint32_t;
using PortId = std::uint32_t;

constexpr NodeId kInvalidNodeId = 0;
constexpr ComponentId kInvalidComponentId = 0;
constexpr PortId kInvalidPortId = 0;

constexpr NodeId kGroundNodeId = 1;

// Circuit quantities use SI base units and double precision, matching the
// simulation core convention.
using Resistance = double;
using Capacitance = double;
using Inductance = double;
using Voltage = double;
using Current = double;

enum class ComponentType
{
    kResistor,
    kCapacitor,
    kInductor,
    kVoltageSource,
    kCurrentSource,
    kDiode,
    kSwitch,
    kDigitalSource,
    kLogicGate,
    kDFlipFlop
};

enum class LogicGateType
{
    kNot,
    kAnd,
    kOr,
    kNand,
    kNor,
    kXor,
    kXnor
};

struct ResistorParameters
{
    Resistance resistance{1.0};
};

struct CapacitorParameters
{
    Capacitance capacitance{1.0};
    Voltage initial_voltage{0.0};
};

struct InductorParameters
{
    Inductance inductance{1.0};
    Current initial_current{0.0};
};

struct VoltageSourceParameters
{
    Voltage voltage{0.0};
};

struct CurrentSourceParameters
{
    Current current{0.0};
};

struct DiodeParameters
{
    Voltage forward_voltage{0.7};
    Resistance on_resistance{1e-3};
    Resistance off_resistance{1e9};
};

struct SwitchParameters
{
    bool closed{false};
    Resistance on_resistance{1e-3};
    Resistance off_resistance{1e9};
};

struct DigitalSourceParameters
{
    bool initial_value{false};
    double frequency_hz{0.0};
};

struct LogicGateParameters
{
    LogicGateType type{LogicGateType::kAnd};
};

struct DFlipFlopParameters
{
    bool initial_q{false};
};

using ComponentParameters = std::variant<
    ResistorParameters,
    CapacitorParameters,
    InductorParameters,
    VoltageSourceParameters,
    CurrentSourceParameters,
    DiodeParameters,
    SwitchParameters,
    DigitalSourceParameters,
    LogicGateParameters,
    DFlipFlopParameters>;

struct ComponentDefinition
{
    ComponentType type{ComponentType::kResistor};
    std::string name;
    std::vector<std::string> port_names;
    ComponentParameters parameters{ResistorParameters{}};
};

struct Port
{
    PortId id{kInvalidPortId};
    std::string name;
    ComponentId component{kInvalidComponentId};
    NodeId node{kInvalidNodeId};
};

struct Node
{
    NodeId id{kInvalidNodeId};
    std::string name;
    std::vector<PortId> ports;
};

struct Component
{
    ComponentId id{kInvalidComponentId};
    ComponentDefinition definition;
    std::vector<PortId> ports;
};

bool matchesType(ComponentType type, const ComponentParameters& parameters);
const char* componentTypeName(ComponentType type);
std::size_t defaultPortCount(ComponentType type);
std::vector<std::string> defaultPortNames(ComponentType type);

}  // namespace srp::circuit
