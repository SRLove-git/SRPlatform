#include "circuit/transient_solver.hpp"

#include "circuit/circuit_model.hpp"
#include "circuit/component_models.hpp"
#include "circuit/linear_solver.hpp"

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

void stampVoltageSource(
    std::vector<std::vector<double>>& matrix,
    std::vector<double>& right_hand_side,
    const std::unordered_map<NodeId, std::size_t>& node_indices,
    NodeId first,
    NodeId second,
    std::size_t source_row,
    double voltage)
{
    if (first != kGroundNodeId)
    {
        matrix[node_indices.at(first)][source_row] += 1.0;
        matrix[source_row][node_indices.at(first)] += 1.0;
    }

    if (second != kGroundNodeId)
    {
        matrix[node_indices.at(second)][source_row] -= 1.0;
        matrix[source_row][node_indices.at(second)] -= 1.0;
    }

    right_hand_side[source_row] = voltage;
}

struct StepSolution
{
    std::vector<double> values;
    std::unordered_map<ComponentId, std::size_t> voltage_source_rows;
    std::unordered_map<ComponentId, std::size_t> capacitor_source_rows;
};

std::optional<StepSolution> solveStep(
    const CircuitModel& circuit,
    const std::unordered_map<NodeId, std::size_t>& node_indices,
    std::size_t non_ground_node_count,
    double time_step,
    bool initial_operating_point,
    const std::unordered_map<ComponentId, double>& capacitor_voltages,
    const std::unordered_map<ComponentId, double>& inductor_currents,
    std::unordered_map<ComponentId, bool>& diode_states)
{
    std::size_t voltage_source_count = 0;
    std::size_t capacitor_count = 0;
    for (const Component& component : circuit.components())
    {
        if (component.definition.type == ComponentType::kVoltageSource)
        {
            ++voltage_source_count;
        }
        else if (component.definition.type == ComponentType::kCapacitor)
        {
            ++capacitor_count;
        }
    }

    const std::size_t unknown_count =
        non_ground_node_count + voltage_source_count +
        (initial_operating_point ? capacitor_count : 0);

    constexpr int kMaxDiodeIterations = 50;
    for (int iteration = 0; iteration < kMaxDiodeIterations; ++iteration)
    {
        std::vector<std::vector<double>> matrix(
            unknown_count,
            std::vector<double>(unknown_count, 0.0));
        std::vector<double> right_hand_side(unknown_count, 0.0);

        StepSolution step_solution;
        std::size_t voltage_source_index = 0;
        std::size_t capacitor_source_index = 0;

        for (const Component& component : circuit.components())
        {
            const NodeId first_node = portNode(circuit, component, 0);
            const NodeId second_node = portNode(circuit, component, 1);

            if (component.definition.type == ComponentType::kResistor)
            {
                const auto& parameters =
                    std::get<ResistorParameters>(component.definition.parameters);
                if (first_node != second_node)
                {
                    stampConductance(
                        matrix,
                        node_indices,
                        first_node,
                        second_node,
                        1.0 / parameters.resistance);
                }
            }
            else if (component.definition.type == ComponentType::kCurrentSource)
            {
                const auto& parameters =
                    std::get<CurrentSourceParameters>(component.definition.parameters);
                stampCurrentSource(
                    right_hand_side,
                    node_indices,
                    first_node,
                    second_node,
                    parameters.current);
            }
            else if (component.definition.type == ComponentType::kVoltageSource)
            {
                const auto& parameters =
                    std::get<VoltageSourceParameters>(component.definition.parameters);
                const std::size_t source_row = non_ground_node_count + voltage_source_index++;
                step_solution.voltage_source_rows[component.id] = source_row;
                stampVoltageSource(
                    matrix,
                    right_hand_side,
                    node_indices,
                    first_node,
                    second_node,
                    source_row,
                    parameters.voltage);
            }
            else if (component.definition.type == ComponentType::kCapacitor)
            {
                const auto& parameters =
                    std::get<CapacitorParameters>(component.definition.parameters);

                if (initial_operating_point)
                {
                    const std::size_t source_row =
                        non_ground_node_count + voltage_source_count + capacitor_source_index++;
                    step_solution.capacitor_source_rows[component.id] = source_row;
                    stampVoltageSource(
                        matrix,
                        right_hand_side,
                        node_indices,
                        first_node,
                        second_node,
                        source_row,
                        parameters.initial_voltage);
                }
                else
                {
                    const double conductance = parameters.capacitance / time_step;
                    stampConductance(
                        matrix,
                        node_indices,
                        first_node,
                        second_node,
                        conductance);
                    stampCurrentSource(
                        right_hand_side,
                        node_indices,
                        first_node,
                        second_node,
                        -conductance * capacitor_voltages.at(component.id));
                }
            }
            else if (component.definition.type == ComponentType::kInductor)
            {
                const auto& parameters =
                    std::get<InductorParameters>(component.definition.parameters);

                if (initial_operating_point)
                {
                    stampCurrentSource(
                        right_hand_side,
                        node_indices,
                        first_node,
                        second_node,
                        parameters.initial_current);
                }
                else
                {
                    const double conductance = time_step / parameters.inductance;
                    stampConductance(
                        matrix,
                        node_indices,
                        first_node,
                        second_node,
                        conductance);
                    stampCurrentSource(
                        right_hand_side,
                        node_indices,
                        first_node,
                        second_node,
                        inductor_currents.at(component.id));
                }
            }
            else if (component.definition.type == ComponentType::kDiode)
            {
                const auto& parameters =
                    std::get<DiodeParameters>(component.definition.parameters);
                const bool conducting = diode_states.at(component.id);
                stampConductance(
                    matrix,
                    node_indices,
                    first_node,
                    second_node,
                    diodeConductance(parameters, conducting));
                stampCurrentSource(
                    right_hand_side,
                    node_indices,
                    first_node,
                    second_node,
                    diodeCurrentSource(parameters, conducting));
            }
            else if (component.definition.type == ComponentType::kSwitch)
            {
                const auto& parameters =
                    std::get<SwitchParameters>(component.definition.parameters);
                stampConductance(
                    matrix,
                    node_indices,
                    first_node,
                    second_node,
                    switchConductance(parameters));
            }
        }

        auto solution = detail::solveLinearSystem(std::move(matrix), std::move(right_hand_side));
        if (!solution.has_value())
        {
            return std::nullopt;
        }

        const auto candidate_voltage_at = [&](NodeId node)
        {
            if (node == kGroundNodeId)
            {
                return 0.0;
            }

            return solution->at(node_indices.at(node));
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

        step_solution.values = std::move(*solution);
        if (!diode_changed)
        {
            return step_solution;
        }
    }

    return std::nullopt;
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

std::vector<ComponentId> sortedComponentIds(const CircuitModel& circuit)
{
    std::vector<ComponentId> ids;
    ids.reserve(circuit.components().size());
    for (const Component& component : circuit.components())
    {
        ids.push_back(component.id);
    }

    std::sort(ids.begin(), ids.end());
    return ids;
}

bool validateDynamicComponent(const Component& component)
{
    if (component.definition.type == ComponentType::kCapacitor)
    {
        const auto& parameters =
            std::get<CapacitorParameters>(component.definition.parameters);
        return finite(parameters.capacitance) && parameters.capacitance > 0.0 &&
               finite(parameters.initial_voltage);
    }

    if (component.definition.type == ComponentType::kInductor)
    {
        const auto& parameters =
            std::get<InductorParameters>(component.definition.parameters);
        return finite(parameters.inductance) && parameters.inductance > 0.0 &&
               finite(parameters.initial_current);
    }

    if (component.definition.type == ComponentType::kResistor)
    {
        const auto& parameters =
            std::get<ResistorParameters>(component.definition.parameters);
        return finite(parameters.resistance) && parameters.resistance > 0.0;
    }

    if (component.definition.type == ComponentType::kVoltageSource)
    {
        const auto& parameters =
            std::get<VoltageSourceParameters>(component.definition.parameters);
        return finite(parameters.voltage);
    }

    if (component.definition.type == ComponentType::kCurrentSource)
    {
        const auto& parameters =
            std::get<CurrentSourceParameters>(component.definition.parameters);
        return finite(parameters.current);
    }

    if (component.definition.type == ComponentType::kDiode)
    {
        const auto& parameters =
            std::get<DiodeParameters>(component.definition.parameters);
        return finite(parameters.forward_voltage) &&
               finite(parameters.on_resistance) &&
               finite(parameters.off_resistance) &&
               parameters.on_resistance > 0.0 &&
               parameters.off_resistance > 0.0;
    }

    if (component.definition.type == ComponentType::kDigitalSource ||
        component.definition.type == ComponentType::kLogicGate ||
        component.definition.type == ComponentType::kDFlipFlop ||
        component.definition.type == ComponentType::kPwmSource)
    {
        return false;
    }

    const auto& parameters =
        std::get<SwitchParameters>(component.definition.parameters);
    return finite(parameters.on_resistance) &&
           finite(parameters.off_resistance) &&
           parameters.on_resistance > 0.0 &&
           parameters.off_resistance > 0.0;
}

}  // namespace

std::optional<Voltage> nodeVoltageAt(
    const TransientResult& result,
    NodeId node,
    std::size_t sample_index)
{
    if (sample_index >= result.times.size())
    {
        return std::nullopt;
    }

    const auto node_it = std::find(result.nodes.begin(), result.nodes.end(), node);
    if (node_it == result.nodes.end())
    {
        return std::nullopt;
    }

    const std::size_t node_index =
        static_cast<std::size_t>(std::distance(result.nodes.begin(), node_it));
    return result.node_voltages[sample_index][node_index];
}

std::optional<Current> componentCurrentAt(
    const TransientResult& result,
    ComponentId component,
    std::size_t sample_index)
{
    if (sample_index >= result.times.size())
    {
        return std::nullopt;
    }

    const auto component_it =
        std::find(result.components.begin(), result.components.end(), component);
    if (component_it == result.components.end())
    {
        return std::nullopt;
    }

    const std::size_t component_index =
        static_cast<std::size_t>(std::distance(result.components.begin(), component_it));
    return result.component_currents[sample_index][component_index];
}

std::optional<TransientResult> simulateTransient(
    const CircuitModel& circuit,
    const TransientSettings& settings)
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

    std::unordered_map<ComponentId, double> capacitor_voltages;
    std::unordered_map<ComponentId, double> inductor_currents;
    std::unordered_map<ComponentId, bool> diode_states;

    for (const Component& component : circuit.components())
    {
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

        if (!validateDynamicComponent(component))
        {
            return std::nullopt;
        }

        if (component.definition.type == ComponentType::kCapacitor)
        {
            capacitor_voltages[component.id] =
                std::get<CapacitorParameters>(component.definition.parameters).initial_voltage;
        }
        else if (component.definition.type == ComponentType::kInductor)
        {
            inductor_currents[component.id] =
                std::get<InductorParameters>(component.definition.parameters).initial_current;
        }
        else if (component.definition.type == ComponentType::kDiode)
        {
            diode_states[component.id] = false;
        }
    }

    const std::size_t sample_count =
        static_cast<std::size_t>(std::llround(settings.end_time / settings.time_step)) + 1;

    TransientResult result;
    result.nodes = sortedNodeIds(circuit);
    result.components = sortedComponentIds(circuit);
    result.times.reserve(sample_count);
    result.node_voltages.reserve(sample_count);
    result.component_currents.reserve(sample_count);

    for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index)
    {
        const bool initial_sample = sample_index == 0;
        const double time =
            sample_index + 1 == sample_count
                ? settings.end_time
                : static_cast<double>(sample_index) * settings.time_step;

        auto step = solveStep(
            circuit,
            node_indices,
            non_ground_node_count,
            settings.time_step,
            initial_sample,
            capacitor_voltages,
            inductor_currents,
            diode_states);
        if (!step.has_value())
        {
            return std::nullopt;
        }

        const auto voltage_at = [&](NodeId node)
        {
            if (node == kGroundNodeId)
            {
                return 0.0;
            }

            return step->values.at(node_indices.at(node));
        };

        result.times.push_back(time);
        std::vector<Voltage> node_row;
        node_row.reserve(result.nodes.size());
        for (const NodeId node_id : result.nodes)
        {
            node_row.push_back(voltage_at(node_id));
        }
        result.node_voltages.push_back(std::move(node_row));

        std::vector<Current> component_row;
        component_row.reserve(result.components.size());
        for (const ComponentId component_id : result.components)
        {
            const Component* component = circuit.component(component_id);
            const NodeId first_node = portNode(circuit, *component, 0);
            const NodeId second_node = portNode(circuit, *component, 1);
            const double first_voltage = voltage_at(first_node);
            const double second_voltage = voltage_at(second_node);
            double current = 0.0;

            if (component->definition.type == ComponentType::kResistor)
            {
                const auto& parameters =
                    std::get<ResistorParameters>(component->definition.parameters);
                current = (first_voltage - second_voltage) / parameters.resistance;
            }
            else if (component->definition.type == ComponentType::kVoltageSource)
            {
                current = step->values.at(step->voltage_source_rows.at(component_id));
            }
            else if (component->definition.type == ComponentType::kCurrentSource)
            {
                current =
                    std::get<CurrentSourceParameters>(component->definition.parameters).current;
            }
            else if (component->definition.type == ComponentType::kCapacitor)
            {
                if (initial_sample)
                {
                    current = step->values.at(step->capacitor_source_rows.at(component_id));
                }
                else
                {
                    const auto& parameters =
                        std::get<CapacitorParameters>(component->definition.parameters);
                    const double conductance = parameters.capacitance / settings.time_step;
                    current =
                        (first_voltage - second_voltage - capacitor_voltages.at(component_id)) *
                        conductance;
                }
            }
            else if (component->definition.type == ComponentType::kInductor)
            {
                if (initial_sample)
                {
                    current = inductor_currents.at(component_id);
                }
                else
                {
                    const auto& parameters =
                        std::get<InductorParameters>(component->definition.parameters);
                    const double conductance = settings.time_step / parameters.inductance;
                    current =
                        (first_voltage - second_voltage) * conductance +
                        inductor_currents.at(component_id);
                }
            }
            else if (component->definition.type == ComponentType::kDiode)
            {
                const auto& parameters =
                    std::get<DiodeParameters>(component->definition.parameters);
                current = diodeCurrent(
                    parameters,
                    diode_states.at(component_id),
                    first_voltage - second_voltage);
            }
            else if (component->definition.type == ComponentType::kSwitch)
            {
                const auto& parameters =
                    std::get<SwitchParameters>(component->definition.parameters);
                current = switchCurrent(parameters, first_voltage - second_voltage);
            }

            component_row.push_back(current);

            if (!initial_sample)
            {
                if (component->definition.type == ComponentType::kCapacitor)
                {
                    capacitor_voltages[component_id] = first_voltage - second_voltage;
                }
                else if (component->definition.type == ComponentType::kInductor)
                {
                    inductor_currents[component_id] = current;
                }
            }
        }

        result.component_currents.push_back(std::move(component_row));
    }

    return result;
}

}  // namespace srp::circuit
