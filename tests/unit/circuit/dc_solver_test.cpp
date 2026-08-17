#include "circuit/circuit_model.hpp"
#include "circuit/dc_solver.hpp"

#include <gtest/gtest.h>

namespace
{

srp::circuit::ComponentId addResistor(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId first,
    srp::circuit::NodeId second,
    double resistance)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kResistor;
    definition.parameters = srp::circuit::ResistorParameters{resistance};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], first);
    circuit.connectPort(component->ports[1], second);
    return id;
}

srp::circuit::ComponentId addVoltageSource(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId positive,
    srp::circuit::NodeId negative,
    double voltage)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kVoltageSource;
    definition.parameters = srp::circuit::VoltageSourceParameters{voltage};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], positive);
    circuit.connectPort(component->ports[1], negative);
    return id;
}

srp::circuit::ComponentId addCurrentSource(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId positive,
    srp::circuit::NodeId negative,
    double current)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kCurrentSource;
    definition.parameters = srp::circuit::CurrentSourceParameters{current};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], positive);
    circuit.connectPort(component->ports[1], negative);
    return id;
}

srp::circuit::ComponentId addCapacitor(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId first,
    srp::circuit::NodeId second,
    double capacitance)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kCapacitor;
    definition.parameters = srp::circuit::CapacitorParameters{capacitance};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], first);
    circuit.connectPort(component->ports[1], second);
    return id;
}

srp::circuit::ComponentId addInductor(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId first,
    srp::circuit::NodeId second,
    double inductance)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kInductor;
    definition.parameters = srp::circuit::InductorParameters{inductance};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], first);
    circuit.connectPort(component->ports[1], second);
    return id;
}

}  // namespace

TEST(DcSolver, SolvesVoltageDivider)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node_a = circuit.addNode("a");
    const srp::circuit::NodeId node_b = circuit.addNode("b");

    const srp::circuit::ComponentId resistor_a_b = addResistor(circuit, node_a, node_b, 1000.0);
    const srp::circuit::ComponentId resistor_b_gnd =
        addResistor(circuit, node_b, srp::circuit::kGroundNodeId, 1000.0);
    const srp::circuit::ComponentId source =
        addVoltageSource(circuit, node_a, srp::circuit::kGroundNodeId, 10.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, srp::circuit::kGroundNodeId), 0.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, node_a), 10.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, node_b), 5.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor_a_b), 0.005, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor_b_gnd), 0.005, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), -0.005, 1e-15);
}

TEST(DcSolver, SolvesCurrentSourceAndResistor)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node_a = circuit.addNode("a");

    const srp::circuit::ComponentId resistor =
        addResistor(circuit, node_a, srp::circuit::kGroundNodeId, 4.0);
    const srp::circuit::ComponentId source =
        addCurrentSource(circuit, node_a, srp::circuit::kGroundNodeId, 2.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, node_a), -8.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), -2.0, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), 2.0, 1e-15);
}

TEST(DcSolver, RejectsNonPositiveResistance)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node_a = circuit.addNode("a");

    addResistor(circuit, node_a, srp::circuit::kGroundNodeId, 0.0);

    EXPECT_FALSE(srp::circuit::solveDc(circuit).has_value());
}

TEST(DcSolver, RejectsDisconnectedPort)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node_a = circuit.addNode("a");

    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kResistor;
    definition.parameters = srp::circuit::ResistorParameters{100.0};

    const srp::circuit::ComponentId resistor = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(resistor);
    circuit.connectPort(component->ports[0], node_a);

    EXPECT_FALSE(srp::circuit::solveDc(circuit).has_value());
}

TEST(DcSolver, RejectsFloatingNode)
{
    srp::circuit::CircuitModel circuit;
    circuit.addNode("floating");

    EXPECT_FALSE(srp::circuit::solveDc(circuit).has_value());
}

TEST(DcSolver, TreatsCapacitorAsOpenCircuitAtDc)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId capacitor_node = circuit.addNode("capacitor");

    const srp::circuit::ComponentId source =
        addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 10.0);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, source_node, capacitor_node, 1000.0);
    const srp::circuit::ComponentId capacitor =
        addCapacitor(circuit, capacitor_node, srp::circuit::kGroundNodeId, 1e-6);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, source_node), 10.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, capacitor_node), 10.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), 0.0, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, capacitor), 0.0, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), 0.0, 1e-15);
}

TEST(DcSolver, TreatsInductorAsShortCircuitAtDc)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId inductor_node = circuit.addNode("inductor");

    const srp::circuit::ComponentId source =
        addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 10.0);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, source_node, inductor_node, 1000.0);
    const srp::circuit::ComponentId inductor =
        addInductor(circuit, inductor_node, srp::circuit::kGroundNodeId, 1.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, source_node), 10.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, inductor_node), 0.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), 0.01, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, inductor), 0.01, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), -0.01, 1e-15);
}

TEST(DcSolver, RejectsNonPositiveCapacitance)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node_a = circuit.addNode("a");

    addCapacitor(circuit, node_a, srp::circuit::kGroundNodeId, 0.0);

    EXPECT_FALSE(srp::circuit::solveDc(circuit).has_value());
}
