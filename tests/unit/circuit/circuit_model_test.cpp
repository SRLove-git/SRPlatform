#include "circuit/circuit_model.hpp"

#include <variant>

#include <gtest/gtest.h>

namespace
{

srp::circuit::ComponentDefinition resistorDefinition(double resistance = 1.0)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kResistor;
    definition.name = "R1";
    definition.parameters = srp::circuit::ResistorParameters{resistance};
    return definition;
}

}  // namespace

TEST(CircuitModel, StartsWithGroundNode)
{
    const srp::circuit::CircuitModel circuit;

    ASSERT_EQ(circuit.nodes().size(), 1);
    EXPECT_TRUE(circuit.isGround(srp::circuit::kGroundNodeId));
    EXPECT_EQ(circuit.node(srp::circuit::kGroundNodeId)->name, "ground");
}

TEST(CircuitModel, AddsNodeWithGeneratedName)
{
    srp::circuit::CircuitModel circuit;

    const srp::circuit::NodeId node_id = circuit.addNode();

    ASSERT_NE(node_id, srp::circuit::kInvalidNodeId);
    const srp::circuit::Node* node = circuit.node(node_id);
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->name, "n" + std::to_string(node_id));
    EXPECT_TRUE(node->ports.empty());
}

TEST(CircuitModel, AddsResistorWithTwoTerminalPorts)
{
    srp::circuit::CircuitModel circuit;

    const srp::circuit::ComponentId component_id =
        circuit.addComponent(resistorDefinition());

    ASSERT_NE(component_id, srp::circuit::kInvalidComponentId);
    const srp::circuit::Component* component = circuit.component(component_id);
    ASSERT_NE(component, nullptr);
    ASSERT_EQ(component->ports.size(), 2);
    EXPECT_EQ(component->definition.port_names.size(), 2);
    EXPECT_EQ(component->definition.port_names[0], "terminal_a");
    EXPECT_EQ(component->definition.port_names[1], "terminal_b");
    EXPECT_EQ(circuit.port(component->ports[0])->component, component_id);
    EXPECT_EQ(circuit.port(component->ports[0])->node, srp::circuit::kInvalidNodeId);
}

TEST(CircuitModel, ConnectsAndReconnectsPortToNode)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId first_node = circuit.addNode("first");
    const srp::circuit::NodeId second_node = circuit.addNode("second");
    const srp::circuit::ComponentId component_id =
        circuit.addComponent(resistorDefinition());
    const srp::circuit::PortId port_id = circuit.component(component_id)->ports[0];

    EXPECT_TRUE(circuit.connectPort(port_id, first_node));
    EXPECT_EQ(circuit.port(port_id)->node, first_node);
    EXPECT_EQ(circuit.node(first_node)->ports.size(), 1);

    EXPECT_TRUE(circuit.connectPort(port_id, second_node));
    EXPECT_EQ(circuit.port(port_id)->node, second_node);
    EXPECT_TRUE(circuit.node(first_node)->ports.empty());
    EXPECT_EQ(circuit.node(second_node)->ports.size(), 1);
}

TEST(CircuitModel, DisconnectsPortFromNode)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node_id = circuit.addNode();
    const srp::circuit::ComponentId component_id =
        circuit.addComponent(resistorDefinition());
    const srp::circuit::PortId port_id = circuit.component(component_id)->ports[0];

    ASSERT_TRUE(circuit.connectPort(port_id, node_id));
    EXPECT_TRUE(circuit.disconnectPort(port_id));
    EXPECT_EQ(circuit.port(port_id)->node, srp::circuit::kInvalidNodeId);
    EXPECT_TRUE(circuit.node(node_id)->ports.empty());
}

TEST(CircuitModel, RejectsMismatchedComponentDefinition)
{
    srp::circuit::CircuitModel circuit;
    srp::circuit::ComponentDefinition definition = resistorDefinition();
    definition.parameters = srp::circuit::VoltageSourceParameters{5.0};

    EXPECT_EQ(circuit.addComponent(definition), srp::circuit::kInvalidComponentId);
    EXPECT_TRUE(circuit.components().empty());
}

TEST(CircuitModel, RejectsDuplicatePortNames)
{
    srp::circuit::CircuitModel circuit;
    srp::circuit::ComponentDefinition definition = resistorDefinition();
    definition.port_names = {"a", "a"};

    EXPECT_EQ(circuit.addComponent(definition), srp::circuit::kInvalidComponentId);
    EXPECT_TRUE(circuit.components().empty());
}

TEST(CircuitModel, RejectsInvalidNodeAndPortConnections)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::ComponentId component_id =
        circuit.addComponent(resistorDefinition());
    const srp::circuit::PortId port_id = circuit.component(component_id)->ports[0];

    EXPECT_FALSE(circuit.connectPort(port_id, srp::circuit::kInvalidNodeId));
    EXPECT_FALSE(circuit.connectPort(srp::circuit::kInvalidPortId, srp::circuit::kGroundNodeId));
    EXPECT_FALSE(circuit.disconnectPort(srp::circuit::kInvalidPortId));
}
