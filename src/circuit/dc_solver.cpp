#include "circuit/dc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

namespace srp::circuit
{

namespace
{

constexpr double kSingularTolerance = 1e-12;

bool finite(double value)
{
    return std::isfinite(value);
}

std::optional<std::vector<double>> solveLinearSystem(
    std::vector<std::vector<double>> matrix,
    std::vector<double> right_hand_side)
{
    const std::size_t size = right_hand_side.size();
    if (size == 0)
    {
        return std::vector<double>{};
    }

    for (std::size_t column = 0; column < size; ++column)
    {
        std::size_t pivot = column;
        double pivot_magnitude = std::abs(matrix[column][column]);
        for (std::size_t row = column + 1; row < size; ++row)
        {
            const double candidate = std::abs(matrix[row][column]);
            if (candidate > pivot_magnitude)
            {
                pivot = row;
                pivot_magnitude = candidate;
            }
        }

        if (pivot_magnitude < kSingularTolerance)
        {
            return std::nullopt;
        }

        if (pivot != column)
        {
            std::swap(matrix[pivot], matrix[column]);
            std::swap(right_hand_side[pivot], right_hand_side[column]);
        }

        for (std::size_t row = column + 1; row < size; ++row)
        {
            const double factor = matrix[row][column] / matrix[column][column];
            matrix[row][column] = 0.0;
            for (std::size_t entry = column + 1; entry < size; ++entry)
            {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
            right_hand_side[row] -= factor * right_hand_side[column];
        }
    }

    std::vector<double> solution(size, 0.0);
    for (std::size_t row = size; row-- > 0;)
    {
        double value = right_hand_side[row];
        for (std::size_t entry = row + 1; entry < size; ++entry)
        {
            value -= matrix[row][entry] * solution[entry];
        }

        solution[row] = value / matrix[row][row];
        if (!finite(solution[row]))
        {
            return std::nullopt;
        }
    }

    return solution;
}

NodeId portNode(const CircuitModel& circuit, const Component& component, std::size_t port_index)
{
    const Port* port = circuit.port(component.ports[port_index]);
    return port == nullptr ? kInvalidNodeId : port->node;
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
    for (const Component& component : circuit.components())
    {
        if (component.definition.type == ComponentType::kCapacitor ||
            component.definition.type == ComponentType::kInductor)
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
    }

    const std::size_t unknown_count = non_ground_node_count + voltage_source_count;
    std::vector<std::vector<double>> matrix(
        unknown_count,
        std::vector<double>(unknown_count, 0.0));
    std::vector<double> right_hand_side(unknown_count, 0.0);
    std::unordered_map<ComponentId, std::size_t> voltage_source_rows;
    std::size_t voltage_source_index = 0;

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

            if (first_node == second_node)
            {
                continue;
            }

            const double conductance = 1.0 / parameters->resistance;
            const bool first_is_ground = first_node == kGroundNodeId;
            const bool second_is_ground = second_node == kGroundNodeId;

            if (!first_is_ground)
            {
                matrix[node_indices[first_node]][node_indices[first_node]] += conductance;
            }

            if (!second_is_ground)
            {
                matrix[node_indices[second_node]][node_indices[second_node]] += conductance;
            }

            if (!first_is_ground && !second_is_ground)
            {
                matrix[node_indices[first_node]][node_indices[second_node]] -= conductance;
                matrix[node_indices[second_node]][node_indices[first_node]] -= conductance;
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

            if (first_node != kGroundNodeId)
            {
                right_hand_side[node_indices[first_node]] -= parameters->current;
            }

            if (second_node != kGroundNodeId)
            {
                right_hand_side[node_indices[second_node]] += parameters->current;
            }
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
    }

    const auto solution = solveLinearSystem(std::move(matrix), std::move(right_hand_side));
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
