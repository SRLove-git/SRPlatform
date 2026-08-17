#include "circuit/circuit_model.hpp"

#include <algorithm>
#include <utility>

namespace srp::circuit
{

namespace
{

std::string generatedNodeName(NodeId id)
{
    return "n" + std::to_string(id);
}

}  // namespace

bool matchesType(ComponentType type, const ComponentParameters& parameters)
{
    switch (type)
    {
    case ComponentType::kResistor:
        return std::holds_alternative<ResistorParameters>(parameters);
    case ComponentType::kCapacitor:
        return std::holds_alternative<CapacitorParameters>(parameters);
    case ComponentType::kInductor:
        return std::holds_alternative<InductorParameters>(parameters);
    case ComponentType::kVoltageSource:
        return std::holds_alternative<VoltageSourceParameters>(parameters);
    case ComponentType::kCurrentSource:
        return std::holds_alternative<CurrentSourceParameters>(parameters);
    case ComponentType::kDiode:
        return std::holds_alternative<DiodeParameters>(parameters);
    case ComponentType::kSwitch:
        return std::holds_alternative<SwitchParameters>(parameters);
    case ComponentType::kDigitalSource:
        return std::holds_alternative<DigitalSourceParameters>(parameters);
    case ComponentType::kLogicGate:
        return std::holds_alternative<LogicGateParameters>(parameters);
    case ComponentType::kDFlipFlop:
        return std::holds_alternative<DFlipFlopParameters>(parameters);
    case ComponentType::kPwmSource:
        return std::holds_alternative<PwmSourceParameters>(parameters);
    }

    return false;
}

const char* componentTypeName(ComponentType type)
{
    switch (type)
    {
    case ComponentType::kResistor:
        return "Resistor";
    case ComponentType::kCapacitor:
        return "Capacitor";
    case ComponentType::kInductor:
        return "Inductor";
    case ComponentType::kVoltageSource:
        return "VoltageSource";
    case ComponentType::kCurrentSource:
        return "CurrentSource";
    case ComponentType::kDiode:
        return "Diode";
    case ComponentType::kSwitch:
        return "Switch";
    case ComponentType::kDigitalSource:
        return "DigitalSource";
    case ComponentType::kLogicGate:
        return "LogicGate";
    case ComponentType::kDFlipFlop:
        return "DFlipFlop";
    case ComponentType::kPwmSource:
        return "PwmSource";
    }

    return "Unknown";
}

std::size_t defaultPortCount(ComponentType type)
{
    switch (type)
    {
    case ComponentType::kResistor:
    case ComponentType::kCapacitor:
    case ComponentType::kInductor:
    case ComponentType::kVoltageSource:
    case ComponentType::kCurrentSource:
    case ComponentType::kDiode:
    case ComponentType::kSwitch:
        return 2;
    case ComponentType::kDigitalSource:
    case ComponentType::kPwmSource:
        return 1;
    case ComponentType::kLogicGate:
        return 3;
    case ComponentType::kDFlipFlop:
        return 4;
    }

    return 0;
}

std::vector<std::string> defaultPortNames(ComponentType type)
{
    switch (type)
    {
    case ComponentType::kResistor:
    case ComponentType::kCapacitor:
    case ComponentType::kInductor:
        return {"terminal_a", "terminal_b"};
    case ComponentType::kVoltageSource:
    case ComponentType::kCurrentSource:
        return {"positive", "negative"};
    case ComponentType::kDiode:
        return {"anode", "cathode"};
    case ComponentType::kSwitch:
        return {"terminal_a", "terminal_b"};
    case ComponentType::kDigitalSource:
    case ComponentType::kPwmSource:
        return {"output"};
    case ComponentType::kLogicGate:
        return {"input_a", "input_b", "output"};
    case ComponentType::kDFlipFlop:
        return {"d", "clock", "q", "qbar"};
    }

    return {};
}

CircuitModel::CircuitModel()
{
    nodes_.push_back(Node{kGroundNodeId, "ground", {}});
    node_indices_[kGroundNodeId] = 0;
}

NodeId CircuitModel::addNode(std::string name)
{
    const NodeId id = next_node_id_++;
    if (name.empty())
    {
        name = generatedNodeName(id);
    }

    node_indices_[id] = nodes_.size();
    nodes_.push_back(Node{id, std::move(name), {}});
    return id;
}

ComponentId CircuitModel::addComponent(const ComponentDefinition& definition)
{
    if (!validComponentDefinition(definition))
    {
        return kInvalidComponentId;
    }

    ComponentDefinition resolved_definition = definition;
    if (resolved_definition.port_names.empty())
    {
        resolved_definition.port_names = defaultPortNames(resolved_definition.type);
    }

    const ComponentId component_id = next_component_id_++;
    Component component;
    component.id = component_id;
    component.definition = std::move(resolved_definition);
    component.ports.reserve(component.definition.port_names.size());

    for (const std::string& port_name : component.definition.port_names)
    {
        const PortId port_id = next_port_id_++;
        port_indices_[port_id] = ports_.size();
        ports_.push_back(Port{port_id, port_name, component_id, kInvalidNodeId});
        component.ports.push_back(port_id);
    }

    component_indices_[component_id] = components_.size();
    components_.push_back(std::move(component));
    return component_id;
}

bool CircuitModel::removeComponent(ComponentId id)
{
    const auto component_it = component_indices_.find(id);
    if (component_it == component_indices_.end())
    {
        return false;
    }

    const std::size_t component_index = component_it->second;
    const std::vector<PortId> removed_ports = components_[component_index].ports;

    // Detach every port of the component from its node first.
    for (const PortId port_id : removed_ports)
    {
        const auto port_it = port_indices_.find(port_id);
        if (port_it != port_indices_.end())
        {
            detachPort(ports_[port_it->second]);
        }
    }

    // Swap-erase the component and its ports, fixing the moved ids.
    const std::size_t last_component = components_.size() - 1;
    if (component_index != last_component)
    {
        components_[component_index] = std::move(components_[last_component]);
        component_indices_[components_[component_index].id] = component_index;
    }
    components_.pop_back();
    component_indices_.erase(id);

    for (const PortId port_id : removed_ports)
    {
        const auto port_it = port_indices_.find(port_id);
        if (port_it == port_indices_.end())
        {
            continue;
        }

        const std::size_t port_index = port_it->second;
        const std::size_t last_port = ports_.size() - 1;
        if (port_index != last_port)
        {
            ports_[port_index] = std::move(ports_[last_port]);
            port_indices_[ports_[port_index].id] = port_index;
        }
        ports_.pop_back();
        port_indices_.erase(port_id);
    }

    pruneEmptyNodes();
    return true;
}

void CircuitModel::pruneEmptyNodes()
{
    for (auto it = node_indices_.begin(); it != node_indices_.end();)
    {
        if (it->first == kGroundNodeId || !nodes_[it->second].ports.empty())
        {
            ++it;
            continue;
        }

        const std::size_t node_index = it->second;
        const std::size_t last_node = nodes_.size() - 1;
        if (node_index != last_node)
        {
            nodes_[node_index] = std::move(nodes_[last_node]);
            node_indices_[nodes_[node_index].id] = node_index;
        }
        nodes_.pop_back();
        it = node_indices_.erase(it);
    }
}

bool CircuitModel::connectPort(PortId port_id, NodeId node_id)
{
    Port* target_port = port(port_id);
    if (target_port == nullptr || node(node_id) == nullptr)
    {
        return false;
    }

    if (target_port->node == node_id)
    {
        return true;
    }

    detachPort(*target_port);
    target_port->node = node_id;
    node(node_id)->ports.push_back(port_id);
    return true;
}

bool CircuitModel::disconnectPort(PortId port_id)
{
    Port* target_port = port(port_id);
    if (target_port == nullptr || target_port->node == kInvalidNodeId)
    {
        return false;
    }

    detachPort(*target_port);
    target_port->node = kInvalidNodeId;
    return true;
}

Node* CircuitModel::node(NodeId id)
{
    const auto it = node_indices_.find(id);
    return it == node_indices_.end() ? nullptr : &nodes_[it->second];
}

const Node* CircuitModel::node(NodeId id) const
{
    const auto it = node_indices_.find(id);
    return it == node_indices_.end() ? nullptr : &nodes_[it->second];
}

Component* CircuitModel::component(ComponentId id)
{
    const auto it = component_indices_.find(id);
    return it == component_indices_.end() ? nullptr : &components_[it->second];
}

const Component* CircuitModel::component(ComponentId id) const
{
    const auto it = component_indices_.find(id);
    return it == component_indices_.end() ? nullptr : &components_[it->second];
}

Port* CircuitModel::port(PortId id)
{
    const auto it = port_indices_.find(id);
    return it == port_indices_.end() ? nullptr : &ports_[it->second];
}

const Port* CircuitModel::port(PortId id) const
{
    const auto it = port_indices_.find(id);
    return it == port_indices_.end() ? nullptr : &ports_[it->second];
}

const std::vector<Node>& CircuitModel::nodes() const
{
    return nodes_;
}

const std::vector<Component>& CircuitModel::components() const
{
    return components_;
}

const std::vector<Port>& CircuitModel::ports() const
{
    return ports_;
}

bool CircuitModel::isGround(NodeId id) const
{
    return id == kGroundNodeId;
}

void CircuitModel::detachPort(Port& port)
{
    if (port.node == kInvalidNodeId)
    {
        return;
    }

    Node* connected_node = node(port.node);
    if (connected_node != nullptr)
    {
        connected_node->ports.erase(
            std::remove(connected_node->ports.begin(), connected_node->ports.end(), port.id),
            connected_node->ports.end());
    }
}

bool CircuitModel::validComponentDefinition(const ComponentDefinition& definition) const
{
    if (!matchesType(definition.type, definition.parameters))
    {
        return false;
    }

    const std::vector<std::string>& port_names = definition.port_names.empty()
                                                     ? defaultPortNames(definition.type)
                                                     : definition.port_names;
    if (port_names.empty())
    {
        return false;
    }

    std::vector<std::string> sorted_names = port_names;
    std::sort(sorted_names.begin(), sorted_names.end());
    return std::adjacent_find(sorted_names.begin(), sorted_names.end()) == sorted_names.end();
}

}  // namespace srp::circuit
