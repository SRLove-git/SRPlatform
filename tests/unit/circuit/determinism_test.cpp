#include "circuit/circuit_model.hpp"
#include "circuit/transient_solver.hpp"

#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

namespace srp::circuit
{
namespace
{

std::uint64_t bits(double value)
{
    std::uint64_t result = 0;
    std::memcpy(&result, &value, sizeof(value));
    return result;
}

CircuitModel buildRcCircuit()
{
    CircuitModel circuit;

    ComponentDefinition source_definition;
    source_definition.type = ComponentType::kVoltageSource;
    source_definition.name = "V1";
    source_definition.port_names = {"positive", "negative"};
    VoltageSourceParameters source_parameters;
    source_parameters.voltage = 12.0;
    source_definition.parameters = source_parameters;
    const ComponentId source = circuit.addComponent(source_definition);

    ComponentDefinition resistor_definition;
    resistor_definition.type = ComponentType::kResistor;
    resistor_definition.name = "R1";
    resistor_definition.port_names = {"terminal_a", "terminal_b"};
    ResistorParameters resistor_parameters;
    resistor_parameters.resistance = 1000.0;
    resistor_definition.parameters = resistor_parameters;
    const ComponentId resistor = circuit.addComponent(resistor_definition);

    ComponentDefinition capacitor_definition;
    capacitor_definition.type = ComponentType::kCapacitor;
    capacitor_definition.name = "C1";
    capacitor_definition.port_names = {"terminal_a", "terminal_b"};
    CapacitorParameters capacitor_parameters;
    capacitor_parameters.capacitance = 1e-3;
    capacitor_definition.parameters = capacitor_parameters;
    const ComponentId capacitor = circuit.addComponent(capacitor_definition);

    const NodeId vcc = circuit.addNode("vcc");
    const NodeId out = circuit.addNode("out");

    EXPECT_TRUE(circuit.connectPort(circuit.component(source)->ports[0], vcc));
    EXPECT_TRUE(circuit.connectPort(circuit.component(source)->ports[1], kGroundNodeId));
    EXPECT_TRUE(circuit.connectPort(circuit.component(resistor)->ports[0], vcc));
    EXPECT_TRUE(circuit.connectPort(circuit.component(resistor)->ports[1], out));
    EXPECT_TRUE(circuit.connectPort(circuit.component(capacitor)->ports[0], out));
    EXPECT_TRUE(circuit.connectPort(circuit.component(capacitor)->ports[1], kGroundNodeId));

    return circuit;
}

void expectResultsEqual(const TransientResult& left, const TransientResult& right)
{
    ASSERT_EQ(left.times.size(), right.times.size());
    ASSERT_EQ(left.node_voltages.size(), right.node_voltages.size());
    ASSERT_EQ(left.component_currents.size(), right.component_currents.size());

    for (std::size_t i = 0; i < left.times.size(); ++i)
    {
        EXPECT_EQ(bits(left.times[i]), bits(right.times[i]));
    }

    for (std::size_t node = 0; node < left.node_voltages.size(); ++node)
    {
        ASSERT_EQ(left.node_voltages[node].size(), right.node_voltages[node].size());
        for (std::size_t i = 0; i < left.node_voltages[node].size(); ++i)
        {
            EXPECT_EQ(
                bits(left.node_voltages[node][i]),
                bits(right.node_voltages[node][i]));
        }
    }

    for (std::size_t component = 0;
         component < left.component_currents.size();
         ++component)
    {
        ASSERT_EQ(
            left.component_currents[component].size(),
            right.component_currents[component].size());
        for (std::size_t i = 0; i < left.component_currents[component].size(); ++i)
        {
            EXPECT_EQ(
                bits(left.component_currents[component][i]),
                bits(right.component_currents[component][i]));
        }
    }
}

TEST(CircuitDeterminismTest, RepeatedTransientRunsMatch)
{
    const CircuitModel circuit = buildRcCircuit();
    TransientSettings settings;
    settings.time_step = 1e-4;
    settings.end_time = 0.02;

    const auto first = simulateTransient(circuit, settings);
    const auto second = simulateTransient(circuit, settings);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    expectResultsEqual(*first, *second);
}

TEST(CircuitDeterminismTest, IdenticalBuildsMatch)
{
    TransientSettings settings;
    settings.time_step = 1e-4;
    settings.end_time = 0.02;

    const auto first = simulateTransient(buildRcCircuit(), settings);
    const auto second = simulateTransient(buildRcCircuit(), settings);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    expectResultsEqual(*first, *second);
}

}  // namespace
}  // namespace srp::circuit
