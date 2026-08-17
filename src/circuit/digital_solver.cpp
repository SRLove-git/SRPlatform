#include "circuit/digital_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace srp::circuit
{

namespace
{

bool finite(double value)
{
    return std::isfinite(value);
}

std::size_t nodeIndex(
    const std::vector<NodeId>& nodes,
    NodeId node)
{
    const auto it = std::find(nodes.begin(), nodes.end(), node);
    return it == nodes.end() ? nodes.size() : static_cast<std::size_t>(std::distance(nodes.begin(), it));
}

std::vector<NodeId> sortedNodeIds(const CircuitModel& circuit)
{
    std::vector<NodeId> ids;
    ids.reserve(circuit.nodes().size());
    for (const Node& node : circuit.nodes())
    {
        ids.push_back(node.id);
    }

    std::sort(ids.begin(), ids.end());
    return ids;
}

bool sourceValue(const DigitalSourceParameters& parameters, double time)
{
    if (parameters.frequency_hz <= 0.0)
    {
        return parameters.initial_value;
    }

    const double half_period = 0.5 / parameters.frequency_hz;
    const auto half_index = static_cast<std::int64_t>(std::floor(time / half_period + 1e-12));
    const bool toggle = (half_index % 2) != 0;
    return parameters.initial_value != toggle;
}

bool pwmValue(const PwmSourceParameters& parameters, double time)
{
    const double phase = std::fmod(time * parameters.frequency_hz, 1.0);
    return phase < parameters.duty_cycle;
}

bool evaluateGate(LogicGateType type, const std::vector<bool>& inputs)
{
    switch (type)
    {
    case LogicGateType::kNot:
        return !inputs.front();
    case LogicGateType::kAnd:
        return std::all_of(inputs.begin(), inputs.end(), [](bool value) { return value; });
    case LogicGateType::kOr:
        return std::any_of(inputs.begin(), inputs.end(), [](bool value) { return value; });
    case LogicGateType::kNand:
        return !std::all_of(inputs.begin(), inputs.end(), [](bool value) { return value; });
    case LogicGateType::kNor:
        return !std::any_of(inputs.begin(), inputs.end(), [](bool value) { return value; });
    case LogicGateType::kXor:
    {
        bool parity = false;
        for (const bool value : inputs)
        {
            parity = parity != value;
        }
        return parity;
    }
    case LogicGateType::kXnor:
    {
        bool parity = false;
        for (const bool value : inputs)
        {
            parity = parity != value;
        }
        return !parity;
    }
    }

    return false;
}

bool validDigitalComponent(const Component& component)
{
    switch (component.definition.type)
    {
    case ComponentType::kDigitalSource:
    {
        const auto& parameters =
            std::get<DigitalSourceParameters>(component.definition.parameters);
        return component.ports.size() == 1 && finite(parameters.frequency_hz) &&
               parameters.frequency_hz >= 0.0;
    }
    case ComponentType::kPwmSource:
    {
        const auto& parameters =
            std::get<PwmSourceParameters>(component.definition.parameters);
        return component.ports.size() == 1 && finite(parameters.frequency_hz) &&
               parameters.frequency_hz > 0.0 && finite(parameters.duty_cycle) &&
               parameters.duty_cycle >= 0.0 && parameters.duty_cycle <= 1.0;
    }
    case ComponentType::kLogicGate:
        return component.ports.size() >= 2;
    case ComponentType::kDFlipFlop:
        return component.ports.size() == 4;
    default:
        return false;
    }
}

}  // namespace

std::optional<bool> nodeValueAt(
    const DigitalResult& result,
    NodeId node,
    std::size_t sample_index)
{
    if (sample_index >= result.times.size())
    {
        return std::nullopt;
    }

    const std::size_t index = nodeIndex(result.nodes, node);
    if (index >= result.nodes.size())
    {
        return std::nullopt;
    }

    return result.node_values[sample_index][index];
}

std::optional<DigitalResult> simulateDigital(
    const CircuitModel& circuit,
    const DigitalSettings& settings)
{
    if (!finite(settings.time_step) || settings.time_step <= 0.0 ||
        !finite(settings.end_time) || settings.end_time < 0.0)
    {
        return std::nullopt;
    }

    if (circuit.node(kGroundNodeId) == nullptr)
    {
        return std::nullopt;
    }

    for (const Component& component : circuit.components())
    {
        if (!validDigitalComponent(component))
        {
            return std::nullopt;
        }

        for (const PortId port_id : component.ports)
        {
            const Port* port = circuit.port(port_id);
            if (port == nullptr || port->node == kInvalidNodeId)
            {
                return std::nullopt;
            }
        }
    }

    const std::vector<NodeId> nodes = sortedNodeIds(circuit);
    const std::size_t sample_count =
        static_cast<std::size_t>(std::llround(settings.end_time / settings.time_step)) + 1;
    const std::size_t max_iterations = circuit.components().size() * 2 + 4;

    std::unordered_map<ComponentId, bool> flip_flop_q;
    std::unordered_map<ComponentId, bool> flip_flop_previous_clock;
    for (const Component& component : circuit.components())
    {
        if (component.definition.type == ComponentType::kDFlipFlop)
        {
            flip_flop_q[component.id] =
                std::get<DFlipFlopParameters>(component.definition.parameters).initial_q;
            flip_flop_previous_clock[component.id] = false;
        }
    }

    DigitalResult result;
    result.nodes = nodes;
    result.times.reserve(sample_count);
    result.node_values.reserve(sample_count);

    for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index)
    {
        const double time =
            sample_index + 1 == sample_count
                ? settings.end_time
                : static_cast<double>(sample_index) * settings.time_step;

        std::vector<bool> values(nodes.size(), false);
        for (std::size_t iteration = 0; iteration < max_iterations; ++iteration)
        {
            bool changed = false;

            for (const Component& component : circuit.components())
            {
                if (component.definition.type == ComponentType::kDigitalSource)
                {
                    const auto& parameters =
                        std::get<DigitalSourceParameters>(component.definition.parameters);
                    const bool value = sourceValue(parameters, time);
                    const std::size_t output = nodeIndex(nodes, circuit.port(component.ports[0])->node);
                    if (values[output] != value)
                    {
                        values[output] = value;
                        changed = true;
                    }
                }
                else if (component.definition.type == ComponentType::kPwmSource)
                {
                    const auto& parameters =
                        std::get<PwmSourceParameters>(component.definition.parameters);
                    const bool value = pwmValue(parameters, time);
                    const std::size_t output = nodeIndex(nodes, circuit.port(component.ports[0])->node);
                    if (values[output] != value)
                    {
                        values[output] = value;
                        changed = true;
                    }
                }
                else if (component.definition.type == ComponentType::kLogicGate)
                {
                    const auto& parameters =
                        std::get<LogicGateParameters>(component.definition.parameters);
                    std::vector<bool> inputs;
                    inputs.reserve(component.ports.size() - 1);
                    for (std::size_t index = 0; index + 1 < component.ports.size(); ++index)
                    {
                        inputs.push_back(values[nodeIndex(nodes, circuit.port(component.ports[index])->node)]);
                    }

                    const bool value = evaluateGate(parameters.type, inputs);
                    const std::size_t output = nodeIndex(
                        nodes,
                        circuit.port(component.ports.back())->node);
                    if (values[output] != value)
                    {
                        values[output] = value;
                        changed = true;
                    }
                }
                else if (component.definition.type == ComponentType::kDFlipFlop)
                {
                    const bool data = values[nodeIndex(nodes, circuit.port(component.ports[0])->node)];
                    const bool clock = values[nodeIndex(nodes, circuit.port(component.ports[1])->node)];

                    if (!flip_flop_previous_clock[component.id] && clock)
                    {
                        flip_flop_q[component.id] = data;
                    }
                    flip_flop_previous_clock[component.id] = clock;

                    const std::size_t q_output = nodeIndex(nodes, circuit.port(component.ports[2])->node);
                    const std::size_t qbar_output = nodeIndex(nodes, circuit.port(component.ports[3])->node);
                    if (values[q_output] != flip_flop_q[component.id])
                    {
                        values[q_output] = flip_flop_q[component.id];
                        changed = true;
                    }

                    const bool qbar = !flip_flop_q[component.id];
                    if (values[qbar_output] != qbar)
                    {
                        values[qbar_output] = qbar;
                        changed = true;
                    }
                }
            }

            if (!changed)
            {
                break;
            }
        }

        result.times.push_back(time);
        result.node_values.push_back(std::move(values));
    }

    return result;
}

}  // namespace srp::circuit
