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

srp::circuit::ComponentId addDiode(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId anode,
    srp::circuit::NodeId cathode,
    double forward_voltage = 0.7)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kDiode;
    definition.parameters = srp::circuit::DiodeParameters{forward_voltage, 1e-3, 1e9};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], anode);
    circuit.connectPort(component->ports[1], cathode);
    return id;
}

srp::circuit::ComponentId addSwitch(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId first,
    srp::circuit::NodeId second,
    bool closed)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kSwitch;
    definition.parameters = srp::circuit::SwitchParameters{closed, 1e-3, 1e9};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], first);
    circuit.connectPort(component->ports[1], second);
    return id;
}

}  // namespace

TEST(CircuitNonlinear, ForwardBiasedDiodeConducts)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId diode_node = circuit.addNode("diode");

    const srp::circuit::ComponentId source =
        addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 5.0);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, source_node, diode_node, 1000.0);
    const srp::circuit::ComponentId diode =
        addDiode(circuit, diode_node, srp::circuit::kGroundNodeId);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, diode_node), 0.7043, 0.01);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, diode), 0.0043, 1e-4);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), 0.0043, 1e-4);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), -0.0043, 1e-4);
}

TEST(CircuitNonlinear, ReverseBiasedDiodeBlocks)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId diode_node = circuit.addNode("diode");

    addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 5.0);
    addResistor(circuit, source_node, diode_node, 1000.0);
    const srp::circuit::ComponentId diode =
        addDiode(circuit, srp::circuit::kGroundNodeId, diode_node);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, diode_node), 5.0, 1e-4);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, diode), 0.0, 1e-6);
}

TEST(CircuitNonlinear, ClosedSwitchConducts)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId load_node = circuit.addNode("load");

    addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 10.0);
    addSwitch(circuit, source_node, load_node, true);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, load_node, srp::circuit::kGroundNodeId, 1000.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, load_node), 10.0, 0.02);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), 0.01, 2e-5);
}

TEST(CircuitNonlinear, OpenSwitchBlocks)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId load_node = circuit.addNode("load");

    addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 10.0);
    addSwitch(circuit, source_node, load_node, false);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, load_node, srp::circuit::kGroundNodeId, 1000.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, load_node), 0.0, 1e-4);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), 0.0, 1e-6);
}
