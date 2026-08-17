#include "circuit/circuit_model.hpp"
#include "circuit/dc_solver.hpp"
#include "circuit/transient_solver.hpp"

#include <limits>

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

}  // namespace

TEST(CircuitSource, VoltageSourceDrivesResistiveLoad)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node = circuit.addNode("output");

    const srp::circuit::ComponentId source =
        addVoltageSource(circuit, node, srp::circuit::kGroundNodeId, 12.0);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, node, srp::circuit::kGroundNodeId, 6.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, node), 12.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), 2.0, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), -2.0, 1e-15);
}

TEST(CircuitSource, CurrentSourceDrivesResistiveLoad)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node = circuit.addNode("output");

    const srp::circuit::ComponentId source =
        addCurrentSource(circuit, node, srp::circuit::kGroundNodeId, 3.0);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, node, srp::circuit::kGroundNodeId, 2.0);

    const auto result = srp::circuit::solveDc(circuit);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*srp::circuit::nodeVoltage(*result, node), -6.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, resistor), -3.0, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrent(*result, source), 3.0, 1e-15);
}

TEST(CircuitSource, CurrentSourceIsHeldInTransientAnalysis)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node = circuit.addNode("output");

    const srp::circuit::ComponentId source =
        addCurrentSource(circuit, node, srp::circuit::kGroundNodeId, 2.0);
    const srp::circuit::ComponentId resistor =
        addResistor(circuit, node, srp::circuit::kGroundNodeId, 4.0);

    srp::circuit::TransientSettings settings;
    settings.time_step = 1e-3;
    settings.end_time = 2e-3;

    const auto result = srp::circuit::simulateTransient(circuit, settings);

    ASSERT_TRUE(result.has_value());
    const std::size_t last = result->times.size() - 1;
    EXPECT_NEAR(*srp::circuit::nodeVoltageAt(*result, node, last), -8.0, 1e-12);
    EXPECT_NEAR(*srp::circuit::componentCurrentAt(*result, resistor, last), -2.0, 1e-15);
    EXPECT_NEAR(*srp::circuit::componentCurrentAt(*result, source, last), 2.0, 1e-15);
}

TEST(CircuitSource, RejectsNonFiniteVoltage)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node = circuit.addNode("output");

    addVoltageSource(
        circuit,
        node,
        srp::circuit::kGroundNodeId,
        std::numeric_limits<double>::quiet_NaN());

    EXPECT_FALSE(srp::circuit::solveDc(circuit).has_value());
}

TEST(CircuitSource, RejectsNonFiniteCurrent)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node = circuit.addNode("output");

    addCurrentSource(
        circuit,
        node,
        srp::circuit::kGroundNodeId,
        std::numeric_limits<double>::infinity());

    EXPECT_FALSE(srp::circuit::solveDc(circuit).has_value());
}
