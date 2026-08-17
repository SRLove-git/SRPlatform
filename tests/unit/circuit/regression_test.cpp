#include "circuit/circuit_model.hpp"
#include "circuit/dc_solver.hpp"
#include "circuit/transient_solver.hpp"

#include <cmath>

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

}  // namespace

TEST(CircuitRegression, ThreeResistorDividerMatchesHandCalculation)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId middle_node = circuit.addNode("middle");
    const srp::circuit::NodeId output_node = circuit.addNode("output");

    const srp::circuit::ComponentId source =
        addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 12.0);
    const srp::circuit::ComponentId r1 =
        addResistor(circuit, source_node, middle_node, 1000.0);
    const srp::circuit::ComponentId r2 =
        addResistor(circuit, middle_node, output_node, 2000.0);
    const srp::circuit::ComponentId r3 =
        addResistor(circuit, output_node, srp::circuit::kGroundNodeId, 3000.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, source_node), 12.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, middle_node), 10.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, output_node), 6.0, 1e-12);

    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, r1), 0.002, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, r2), 0.002, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, r3), 0.002, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), -0.002, 1e-15);
}

TEST(CircuitRegression, TwoSourcesAcrossResistorMatchHandCalculation)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId left = circuit.addNode("left");
    const srp::circuit::NodeId right = circuit.addNode("right");

    const srp::circuit::ComponentId source_left =
        addVoltageSource(circuit, left, srp::circuit::kGroundNodeId, 10.0);
    const srp::circuit::ComponentId source_right =
        addVoltageSource(circuit, right, srp::circuit::kGroundNodeId, 5.0);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, left, right, 1000.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, left), 10.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, right), 5.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), 0.005, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source_left), -0.005, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source_right), 0.005, 1e-15);
}

TEST(CircuitRegression, RcTimeConstantMatchesAnalyticSolution)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId capacitor_node = circuit.addNode("capacitor");

    addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 10.0);
    addResistor(circuit, source_node, capacitor_node, 1000.0);
    addCapacitor(circuit, capacitor_node, srp::circuit::kGroundNodeId, 1e-6);

    srp::circuit::TransientSettings settings;
    settings.time_step = 1e-5;
    settings.end_time = 3e-3;

    const auto result = srp::circuit::simulateTransient(circuit, settings);

    ASSERT_TRUE(result.has_value());
    const std::size_t last = result->times.size() - 1;
    const double expected = 10.0 * (1.0 - std::exp(-3.0));
    EXPECT_NEAR(
        *srp::circuit::nodeVoltageAt(*result, capacitor_node, last),
        expected,
        0.01);
}
