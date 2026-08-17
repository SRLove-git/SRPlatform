#pragma once

#include "circuit/circuit_types.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace srp::circuit
{

class CircuitModel
{
public:
    CircuitModel();

    NodeId addNode(std::string name = {});
    ComponentId addComponent(const ComponentDefinition& definition);

    // Removes a component, detaches its ports, and prunes nodes that no
    // longer have any port (the ground node is never pruned). Returns false
    // when the id does not exist.
    bool removeComponent(ComponentId id);

    bool connectPort(PortId port_id, NodeId node_id);
    bool disconnectPort(PortId port_id);

    // Removes nodes that have no connected port (the ground node is kept).
    void pruneEmptyNodes();

    Node* node(NodeId id);
    const Node* node(NodeId id) const;

    Component* component(ComponentId id);
    const Component* component(ComponentId id) const;

    Port* port(PortId id);
    const Port* port(PortId id) const;

    const std::vector<Node>& nodes() const;
    const std::vector<Component>& components() const;
    const std::vector<Port>& ports() const;

    bool isGround(NodeId id) const;

private:
    std::vector<Node> nodes_;
    std::vector<Component> components_;
    std::vector<Port> ports_;

    std::unordered_map<NodeId, std::size_t> node_indices_;
    std::unordered_map<ComponentId, std::size_t> component_indices_;
    std::unordered_map<PortId, std::size_t> port_indices_;

    NodeId next_node_id_{kGroundNodeId + 1};
    ComponentId next_component_id_{kInvalidComponentId + 1};
    PortId next_port_id_{kInvalidPortId + 1};

    void detachPort(Port& port);
    bool validComponentDefinition(const ComponentDefinition& definition) const;
};

}  // namespace srp::circuit
