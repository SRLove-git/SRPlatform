#include "circuit/dc_solver.hpp"

#include "circuit/component_models.hpp"
#include "circuit/linear_solver.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace srp::circuit
{

namespace
{

bool finite(double value)
{
    return std::isfinite(value);
}

NodeId portNode(const CircuitModel& circuit, const Component& component, std::size_t port_index)
{
    const Port* port = circuit.port(component.ports[port_index]);
    return port == nullptr ? kInvalidNodeId : port->node;
}

void stampConductance(
    std::vector<std::vector<double>>& matrix,
    const std::unordered_map<NodeId, std::size_t>& node_indices,
    NodeId first,
    NodeId second,
    double conductance)
{
    const bool first_is_ground = first == kGroundNodeId;
    const bool second_is_ground = second == kGroundNodeId;

    if (!first_is_ground)
    {
        matrix[node_indices.at(first)][node_indices.at(first)] += conductance;
    }

    if (!second_is_ground)
    {
        matrix[node_indices.at(second)][node_indices.at(second)] += conductance;
    }

    if (!first_is_ground && !second_is_ground)
    {
        matrix[node_indices.at(first)][node_indices.at(second)] -= conductance;
        matrix[node_indices.at(second)][node_indices.at(first)] -= conductance;
    }
}

void stampCurrentSource(
    std::vector<double>& right_hand_side,
    const std::unordered_map<NodeId, std::size_t>& node_indices,
    NodeId first,
    NodeId second,
    double current)
{
    if (first != kGroundNodeId)
    {
        right_hand_side[node_indices.at(first)] -= current;
    }

    if (second != kGroundNodeId)
    {
        right_hand_side[node_indices.at(second)] += current;
    }
}

}  // namespace

std::optional<Voltage> nodeVoltage(const DcAnalysisResult& result, NodeId node)
{
    const auto it = std::find_if(
        result.node_voltages.begin(),
        result.node_voltages.end(),
        [node](const DcNodeVoltage& value) { return value.node == node; });

    if (it == result.node_voltages.end())
    {
        return std::nullopt;
    }

    return it->voltage;
}

std::optional<Current> componentCurrent(
    const DcAnalysisResult& result,
    ComponentId component)
{
    const auto it = std::find_if(
        result.component_currents.begin(),
        result.component_currents.end(),
        [component](const DcComponentCurrent& value)
        { return value.component == component; });

    if (it == result.component_currents.end())
    {
        return std::nullopt;
    }

    return it->current;
}

std::optional<DcAnalysisResult> solveDc(const CircuitModel& circuit)
{
    const Node* ground = circuit.node(kGroundNodeId);
    if (ground == nullptr)
    {
        return std::nullopt;
    }

    std::unordered_map<NodeId, std::size_t> node_indices;
    std::size_t non_ground_node_count = 0;

    for (const Node& node : circuit.nodes())
    {
        if (node.id == kGroundNodeId)
        {
            continue;
        }

        if (node.ports.empty())
        {
            return std::nullopt;
        }

        node_indices[node.id] = non_ground_node_count++;
    }

    std::size_t voltage_source_count = 0;
    std::size_t inductor_count = 0;
    for (const Component& component : circuit.components())
    {
        if (component.definition.type == ComponentType::kDigitalSource ||
            component.definition.type == ComponentType::kLogicGate ||
            component.definition.type == ComponentType::kDFlipFlop)
        {
            return std::nullopt;
        }

        if (component.ports.size() != 2)
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

        if (component.definition.type == ComponentType::kVoltageSource)
        {
            ++voltage_source_count;
        }
        else if (component.definition.type == ComponentType::kInductor)
        {
            ++inductor_count;
        }
    }

    std::unordered_map<ComponentId, bool> diode_states;
    for (const Component& component : circuit.components())
    {
        if (component.definition.type == ComponentType::kDiode)
        {
            diode_states[component.id] = false;
        }
    }

    const std::size_t unknown_count =
        non_ground_node_count + voltage_source_count + inductor_count;
    constexpr int kMaxDiodeIterations = 50;

    std::optional<std::vector<double>> solution;
    std::vector<std::vector<double>> matrix;
    std::vector<double> right_hand_side;
    std::unordered_map<ComponentId, std::size_t> voltage_source_rows;
    std::unordered_map<ComponentId, std::size_t> inductor_rows;

    for (int iteration = 0; iteration < kMaxDiodeIterations; ++iteration)
    {
        matrix.assign(unknown_count, std::vector<double>(unknown_count, 0.0));
        right_hand_side.assign(unknown_count, 0.0);
        voltage_source_rows.clear();
        inductor_rows.clear();

        std::size_t voltage_source_index = 0;
        std::size_t inductor_index = 0;

        for (const Component& component : circuit.components())
        {
            const NodeId first_node = portNode(circuit, component, 0);
            const NodeId second_node = portNode(circuit, component, 1);

            if (component.definition.type == ComponentType::kResistor)
            {
                const auto* parameters =
                    std::get_if<ResistorParameters>(&component.definition.parameters);
                if (parameters == nullptr || !finite(parameters->resistance) ||
                    parameters->resistance <= 0.0)
                {
                    return std::nullopt;
                }

                if (first_node != second_node)
                {
                    stampConductance(
                        matrix,
                        node_indices,
                        first_node,
                        second_node,
                        1.0 / parameters->resistance);
                }
            }
            else if (component.definition.type == ComponentType::kCurrentSource)
            {
                const auto* parameters =
                    std::get_if<CurrentSourceParameters>(&component.definition.parameters);
                if (parameters == nullptr || !finite(parameters->current))
                {
                    return std::nullopt;
                }

                stampCurrentSource(
                    right_hand_side,
                    node_indices,
                    first_node,
                    second_node,
                    parameters->current);
            }
            else if (component.definition.type == ComponentType::kVoltageSource)
            {
                const auto* parameters =
                    std::get_if<VoltageSourceParameters>(&component.definition.parameters);
                if (parameters == nullptr || !finite(parameters->voltage))
                {
                    return std::nullopt;
                }

                const std::size_t source_row = non_ground_node_count + voltage_source_index++;
                voltage_source_rows[component.id] = source_row;

                if (first_node != kGroundNodeId)
                {
                    matrix[node_indices[first_node]][source_row] += 1.0;
                    matrix[source_row][node_indices[first_node]] += 1.0;
                }

                if (second_node != kGroundNodeId)
                {
                    matrix[node_indices[second_node]][source_row] -= 1.0;
                    matrix[source_row][node_indices[second_node]] -= 1.0;
                }

                right_hand_side[source_row] = parameters->voltage;
            }
            else if (component.definition.type == ComponentType::kCapacitor)
            {
                const auto* parameters =
                    std::get_if<CapacitorParameters>(&component.definition.parameters);
                if (parameters == nullptr || !finite(parameters->capacitance) ||
                    parameters->capacitance <= 0.0)
                {
                    return std::nullopt;
                }
            }
            else if (component.definition.type == ComponentType::kInductor)
            {
                const auto* parameters =
                    std::get_if<InductorParameters>(&component.definition.parameters);
                if (parameters == nullptr || !finite(parameters->inductance) ||
                    parameters->inductance <= 0.0)
                {
                    return std::nullopt;
                }

                const std::size_t source_row =
                    non_ground_node_count + voltage_source_count + inductor_index++;
                inductor_rows[component.id] = source_row;

                if (first_node != kGroundNodeId)
                {
                    matrix[node_indices[first_node]][source_row] += 1.0;
                    matrix[source_row][node_indices[first_node]] += 1.0;
                }

                if (second_node != kGroundNodeId)
                {
                    matrix[node_indices[second_node]][source_row] -= 1.0;
                    matrix[source_row][node_indices[second_node]] -= 1.0;
                }

                right_hand_side[source_row] = 0.0;
            }
            else if (component.definition.type == ComponentType::kDiode)
            {
                const auto* parameters =
                    std::get_if<DiodeParameters>(&component.definition.parameters);
                if (parameters == nullptr || !finite(parameters->forward_voltage) ||
                    !finite(parameters->on_resistance) ||
                    !finite(parameters->off_resistance) ||
                    parameters->on_resistance <= 0.0 ||
                    parameters->off_resistance <= 0.0)
                {
                    return std::nullopt;
                }

                const bool conducting = diode_states.at(component.id);
                stampConductance(
                    matrix,
                    node_indices,
                    first_node,
                    second_node,
                    diodeConductance(*parameters, conducting));
                stampCurrentSource(
                    right_hand_side,
                    node_indices,
                    first_node,
                    second_node,
                    diodeCurrentSource(*parameters, conducting));
            }
            else if (component.definition.type == ComponentType::kSwitch)
            {
                const auto* parameters =
                    std::get_if<SwitchParameters>(&component.definition.parameters);
                if (parameters == nullptr || !finite(parameters->on_resistance) ||
                    !finite(parameters->off_resistance) ||
                    parameters->on_resistance <= 0.0 ||
                    parameters->off_resistance <= 0.0)
                {
                    return std::nullopt;
                }

                stampConductance(
                    matrix,
                    node_indices,
                    first_node,
                    second_node,
                    switchConductance(*parameters));
            }
        }

        auto candidate =
            detail::solveLinearSystem(std::move(matrix), std::move(right_hand_side));
        if (!candidate.has_value())
        {
            return std::nullopt;
        }

        const auto candidate_voltage_at = [&](NodeId node)
        {
            if (node == kGroundNodeId)
            {
                return 0.0;
            }

            return candidate->at(node_indices.at(node));
        };

        bool diode_changed = false;
        for (const Component& component : circuit.components())
        {
            if (component.definition.type != ComponentType::kDiode)
            {
                continue;
            }

            const auto& parameters =
                std::get<DiodeParameters>(component.definition.parameters);
            const NodeId first_node = portNode(circuit, component, 0);
            const NodeId second_node = portNode(circuit, component, 1);
            const Voltage branch_voltage =
                candidate_voltage_at(first_node) - candidate_voltage_at(second_node);
            const bool conducting = diodeConducts(branch_voltage, parameters);
            if (conducting != diode_states[component.id])
            {
                diode_states[component.id] = conducting;
                diode_changed = true;
            }
        }

        solution = std::move(candidate);
        if (!diode_changed)
        {
            break;
        }
    }

    if (!solution.has_value())
    {
        return std::nullopt;
    }

    const auto voltage_at = [&](NodeId node)
    {
        if (node == kGroundNodeId)
        {
            return 0.0;
        }

        return solution->at(node_indices.at(node));
    };

    DcAnalysisResult result;
    for (const Node& node : circuit.nodes())
    {
        result.node_voltages.push_back(DcNodeVoltage{
            node.id,
            voltage_at(node.id)});
    }

    std::sort(
        result.node_voltages.begin(),
        result.node_voltages.end(),
        [](const DcNodeVoltage& lhs, const DcNodeVoltage& rhs)
        { return lhs.node < rhs.node; });

    for (const Component& component : circuit.components())
    {
        double current = 0.0;
        const NodeId first_node = portNode(circuit, component, 0);
        const NodeId second_node = portNode(circuit, component, 1);
        const double first_voltage = voltage_at(first_node);
        const double second_voltage = voltage_at(second_node);

        if (component.definition.type == ComponentType::kResistor)
        {
            const auto& parameters =
                std::get<ResistorParameters>(component.definition.parameters);
            current = (first_voltage - second_voltage) / parameters.resistance;
        }
        else if (component.definition.type == ComponentType::kVoltageSource)
        {
            current = solution->at(voltage_source_rows.at(component.id));
        }
        else if (component.definition.type == ComponentType::kCurrentSource)
        {
            current = std::get<CurrentSourceParameters>(component.definition.parameters).current;
        }
        else if (component.definition.type == ComponentType::kCapacitor)
        {
            current = 0.0;
        }
        else if (component.definition.type == ComponentType::kInductor)
        {
            current = solution->at(inductor_rows.at(component.id));
        }
        else if (component.definition.type == ComponentType::kDiode)
        {
            const auto& parameters =
                std::get<DiodeParameters>(component.definition.parameters);
            current = diodeCurrent(
                parameters,
                diode_states.at(component.id),
                first_voltage - second_voltage);
        }
        else if (component.definition.type == ComponentType::kSwitch)
        {
            const auto& parameters =
                std::get<SwitchParameters>(component.definition.parameters);
            current = switchCurrent(parameters, first_voltage - second_voltage);
        }

        result.component_currents.push_back(DcComponentCurrent{component.id, current});
    }

    std::sort(
        result.component_currents.begin(),
        result.component_currents.end(),
        [](const DcComponentCurrent& lhs, const DcComponentCurrent& rhs)
        { return lhs.component < rhs.component; });

    return result;
}

}  // namespace srp::circuit
