#include "circuit/circuit_model.hpp"
#include "circuit/digital_solver.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace
{

srp::circuit::ComponentId addDigitalSource(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId output,
    bool initial_value,
    double frequency_hz = 0.0)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kDigitalSource;
    definition.parameters =
        srp::circuit::DigitalSourceParameters{initial_value, frequency_hz};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], output);
    return id;
}

srp::circuit::ComponentId addLogicGate(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::LogicGateType type,
    srp::circuit::NodeId input_a,
    srp::circuit::NodeId input_b,
    srp::circuit::NodeId output)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kLogicGate;
    definition.parameters = srp::circuit::LogicGateParameters{type};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], input_a);
    circuit.connectPort(component->ports[1], input_b);
    circuit.connectPort(component->ports[2], output);
    return id;
}

srp::circuit::ComponentId addPwmSource(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId output,
    double frequency_hz,
    double duty_cycle)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kPwmSource;
    definition.parameters =
        srp::circuit::PwmSourceParameters{frequency_hz, duty_cycle};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], output);
    return id;
}

srp::circuit::ComponentId addDFlipFlop(
    srp::circuit::CircuitModel& circuit,
    srp::circuit::NodeId data,
    srp::circuit::NodeId clock,
    srp::circuit::NodeId q,
    srp::circuit::NodeId qbar,
    bool initial_q = false)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kDFlipFlop;
    definition.parameters = srp::circuit::DFlipFlopParameters{initial_q};

    const srp::circuit::ComponentId id = circuit.addComponent(definition);
    const srp::circuit::Component* component = circuit.component(id);
    circuit.connectPort(component->ports[0], data);
    circuit.connectPort(component->ports[1], clock);
    circuit.connectPort(component->ports[2], q);
    circuit.connectPort(component->ports[3], qbar);
    return id;
}

std::size_t timeIndex(const srp::circuit::DigitalResult& result, double time)
{
    for (std::size_t index = 0; index < result.times.size(); ++index)
    {
        if (std::abs(result.times[index] - time) < 1e-12)
        {
            return index;
        }
    }

    return result.times.size();
}

}  // namespace

TEST(DigitalSolver, AndGateEvaluatesInputs)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId a = circuit.addNode("a");
    const srp::circuit::NodeId b = circuit.addNode("b");
    const srp::circuit::NodeId y = circuit.addNode("y");

    addDigitalSource(circuit, a, true);
    addDigitalSource(circuit, b, true);
    addLogicGate(circuit, srp::circuit::LogicGateType::kAnd, a, b, y);

    srp::circuit::DigitalSettings settings;
    settings.time_step = 1e-3;
    settings.end_time = 1e-3;

    const auto result = srp::circuit::simulateDigital(circuit, settings);

    ASSERT_TRUE(result.has_value());
    const std::size_t last = result->times.size() - 1;
    EXPECT_TRUE(*srp::circuit::nodeValueAt(*result, y, last));
}

TEST(DigitalSolver, XorGateEvaluatesInputs)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId a = circuit.addNode("a");
    const srp::circuit::NodeId b = circuit.addNode("b");
    const srp::circuit::NodeId y = circuit.addNode("y");

    addDigitalSource(circuit, a, true);
    addDigitalSource(circuit, b, false);
    addLogicGate(circuit, srp::circuit::LogicGateType::kXor, a, b, y);

    srp::circuit::DigitalSettings settings;
    settings.time_step = 1e-3;
    settings.end_time = 1e-3;

    const auto result = srp::circuit::simulateDigital(circuit, settings);

    ASSERT_TRUE(result.has_value());
    const std::size_t last = result->times.size() - 1;
    EXPECT_TRUE(*srp::circuit::nodeValueAt(*result, y, last));
}

TEST(DigitalSolver, DFlipFlopLatchesOnRisingEdge)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId data = circuit.addNode("data");
    const srp::circuit::NodeId clock = circuit.addNode("clock");
    const srp::circuit::NodeId q = circuit.addNode("q");
    const srp::circuit::NodeId qbar = circuit.addNode("qbar");

    addDigitalSource(circuit, data, true);
    addDigitalSource(circuit, clock, false, 1.0);
    addDFlipFlop(circuit, data, clock, q, qbar);

    srp::circuit::DigitalSettings settings;
    settings.time_step = 0.1;
    settings.end_time = 0.6;

    const auto result = srp::circuit::simulateDigital(circuit, settings);

    ASSERT_TRUE(result.has_value());
    const std::size_t before_edge = timeIndex(*result, 0.4);
    const std::size_t after_edge = timeIndex(*result, 0.5);
    ASSERT_LT(before_edge, result->times.size());
    ASSERT_LT(after_edge, result->times.size());

    EXPECT_FALSE(*srp::circuit::nodeValueAt(*result, q, before_edge));
    EXPECT_TRUE(*srp::circuit::nodeValueAt(*result, q, after_edge));
    EXPECT_TRUE(*srp::circuit::nodeValueAt(*result, qbar, before_edge));
    EXPECT_FALSE(*srp::circuit::nodeValueAt(*result, qbar, after_edge));
}

TEST(DigitalSolver, DFlipFlopKeepsStateWithoutClockEdge)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId data = circuit.addNode("data");
    const srp::circuit::NodeId clock = circuit.addNode("clock");
    const srp::circuit::NodeId q = circuit.addNode("q");
    const srp::circuit::NodeId qbar = circuit.addNode("qbar");

    addDigitalSource(circuit, data, true);
    addDigitalSource(circuit, clock, false);
    addDFlipFlop(circuit, data, clock, q, qbar);

    srp::circuit::DigitalSettings settings;
    settings.time_step = 1e-3;
    settings.end_time = 2e-3;

    const auto result = srp::circuit::simulateDigital(circuit, settings);

    ASSERT_TRUE(result.has_value());
    const std::size_t last = result->times.size() - 1;
    EXPECT_FALSE(*srp::circuit::nodeValueAt(*result, q, last));
    EXPECT_TRUE(*srp::circuit::nodeValueAt(*result, qbar, last));
}

TEST(DigitalSolver, PwmSourceHonorsDutyCycle)
{
    srp::circuit::CircuitModel circuit;
    const srp::circuit::NodeId pwm = circuit.addNode("pwm");

    addPwmSource(circuit, pwm, 1000.0, 0.25);

    srp::circuit::DigitalSettings settings;
    settings.time_step = 1e-4;
    settings.end_time = 1e-3;

    const auto result = srp::circuit::simulateDigital(circuit, settings);

    ASSERT_TRUE(result.has_value());
    const std::size_t high_sample = timeIndex(*result, 1e-4);
    const std::size_t low_sample = timeIndex(*result, 3e-4);
    ASSERT_LT(high_sample, result->times.size());
    ASSERT_LT(low_sample, result->times.size());

    EXPECT_TRUE(*srp::circuit::nodeValueAt(*result, pwm, high_sample));
    EXPECT_FALSE(*srp::circuit::nodeValueAt(*result, pwm, low_sample));
}
