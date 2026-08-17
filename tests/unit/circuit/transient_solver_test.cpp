#include "circuit/circuit_model.hpp"
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
    double capacitance,
    double initial_voltage = 0.0)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kCapacitor;
    definition.parameters = srp::circuit::CapacitorParameters{capacitance, initial_voltage};

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
    double inductance,
    double initial_current = 0.0)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kInductor;
    definition.parameters = srp::circuit::InductorParameters{inductance, initial_current};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], first);
    circuit.connectPort(component->ports[1], second);
    return id;
}

}  // namespace

TEST(TransientSolver, SolvesRcChargingWaveform)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId capacitor_node = circuit.addNode("capacitor");

    addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 10.0);
    addResistor(circuit, source_node, capacitor_node, 1000.0);
    addCapacitor(circuit, capacitor_node, srp::circuit::kGroundNodeId, 1e-6);

    srp::circuit::TransientSettings settings;
    settings.time_step = 1e-5;
    settings.end_time = 5e-3;

    const auto result = srp::circuit::simulateTransient(circuit, settings);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->times.size(), 501);
    EXPECT_DOUBLE_EQ(result->times.front(), 0.0);
    EXPECT_DOUBLE_EQ(result->times.back(), 5e-3);
    EXPECT_NEAR(
        *srp::circuit::nodeVoltageAt(*result, capacitor_node, 0),
        0.0,
        1e-12);

    const double expected = 10.0 * (1.0 - std::exp(-5.0));
    EXPECT_NEAR(
        *srp::circuit::nodeVoltageAt(*result, capacitor_node, result->times.size() - 1),
        expected,
        0.01);
}

TEST(TransientSolver, SolvesRlChargingWaveform)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId source_node = circuit.addNode("source");
    const srp::circuit::NodeId inductor_node = circuit.addNode("inductor");

    addVoltageSource(circuit, source_node, srp::circuit::kGroundNodeId, 10.0);
    addResistor(circuit, source_node, inductor_node, 1000.0);
    const srp::circuit::ComponentId inductor =
        addInductor(circuit, inductor_node, srp::circuit::kGroundNodeId, 1.0);

    srp::circuit::TransientSettings settings;
    settings.time_step = 1e-5;
    settings.end_time = 5e-3;

    const auto result = srp::circuit::simulateTransient(circuit, settings);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(
        *srp::circuit::componentCurrentAt(*result, inductor, 0),
        0.0,
        1e-12);

    const double expected = 0.01 * (1.0 - std::exp(-5.0));
    EXPECT_NEAR(
        *srp::circuit::componentCurrentAt(
            *result,
            inductor,
            result->times.size() - 1),
        expected,
        1e-4);
}

TEST(TransientSolver, RejectsInvalidTimeStep)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node = circuit.addNode("a");
    addResistor(circuit, node, srp::circuit::kGroundNodeId, 100.0);

    srp::circuit::TransientSettings settings;
    settings.time_step = 0.0;
    settings.end_time = 1.0;

    EXPECT_FALSE(srp::circuit::simulateTransient(circuit, settings).has_value());
}

TEST(TransientSolver, RejectsNonPositiveCapacitance)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId node = circuit.addNode("a");
    addCapacitor(circuit, node, srp::circuit::kGroundNodeId, 0.0);

    srp::circuit::TransientSettings settings;
    settings.time_step = 1e-4;
    settings.end_time = 1e-3;

    EXPECT_FALSE(srp::circuit::simulateTransient(circuit, settings).has_value());
}
