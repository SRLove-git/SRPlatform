#include "circuit/circuit_model.hpp"
#include "circuit/dc_solver.hpp"
#include "circuit/digital_solver.hpp"
#include "circuit/transient_solver.hpp"
#include "physics/physics_world.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace
{

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(Clock::time_point start, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void printRow(
    const std::string& name,
    std::size_t units,
    double milliseconds_per_operation)
{
    std::cout << std::left << std::setw(28) << name
              << std::right << std::setw(10) << units
              << std::setw(18) << std::fixed << std::setprecision(4)
              << milliseconds_per_operation
              << std::setw(14) << std::setprecision(1)
              << (milliseconds_per_operation > 0.0
                      ? 1000.0 / milliseconds_per_operation
                      : 0.0)
              << "\n";
}

srp::physics::PhysicsWorld buildPhysicsScene(std::size_t body_count)
{
    srp::physics::PhysicsWorld world;

    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;
    srp::physics::PlaneShape ground;
    ground.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    world.createBody(ground_state, ground);

    for (std::size_t i = 0; i < body_count; ++i)
    {
        srp::physics::RigidBodyState state;
        state.type = srp::physics::RigidBodyType::kDynamic;
        state.mass = 1.0;
        const std::size_t columns = static_cast<std::size_t>(
            std::ceil(std::sqrt(static_cast<double>(body_count))));
        const double x = static_cast<double>(i % columns) * 1.3 - 2.0;
        const double z = static_cast<double>(i / columns) * 1.3 - 2.0;
        const double y = 4.0 + static_cast<double>(i % 5) * 1.5;
        state.position = srp::math::Vec3(x, y, z);
        state.restitution = 0.1;
        state.friction = 0.6;

        if (i % 3 == 0)
        {
            srp::physics::SphereShape sphere;
            sphere.radius = 0.35;
            world.createBody(state, sphere);
        }
        else
        {
            srp::physics::BoxShape box;
            box.half_extents = srp::math::Vec3(0.35);
            world.createBody(state, box);
        }
    }
    return world;
}

double benchPhysics(std::size_t body_count, std::size_t steps)
{
    srp::physics::PhysicsWorld world = buildPhysicsScene(body_count);
    constexpr double kDt = 1.0 / 60.0;

    for (std::size_t i = 0; i < 30; ++i)
    {
        world.step(kDt);
    }

    const auto start = Clock::now();
    for (std::size_t i = 0; i < steps; ++i)
    {
        world.step(kDt);
    }
    const auto end = Clock::now();
    return elapsedMilliseconds(start, end) / static_cast<double>(steps);
}

srp::circuit::CircuitModel buildLadderCircuit(std::size_t node_count)
{
    srp::circuit::CircuitModel circuit;

    srp::circuit::ComponentDefinition source_definition;
    source_definition.type = srp::circuit::ComponentType::kVoltageSource;
    source_definition.name = "V1";
    source_definition.port_names = {"positive", "negative"};
    srp::circuit::VoltageSourceParameters source_parameters;
    source_parameters.voltage = 12.0;
    source_definition.parameters = source_parameters;
    const srp::circuit::ComponentId source = circuit.addComponent(source_definition);

    std::vector<srp::circuit::NodeId> nodes;
    for (std::size_t i = 0; i < node_count; ++i)
    {
        nodes.push_back(circuit.addNode("n" + std::to_string(i)));
    }

    circuit.connectPort(circuit.component(source)->ports[0], nodes.front());
    circuit.connectPort(circuit.component(source)->ports[1], srp::circuit::kGroundNodeId);

    for (std::size_t i = 0; i < node_count; ++i)
    {
        srp::circuit::ComponentDefinition resistor_definition;
        resistor_definition.type = srp::circuit::ComponentType::kResistor;
        resistor_definition.name = "R" + std::to_string(i);
        resistor_definition.port_names = {"terminal_a", "terminal_b"};
        srp::circuit::ResistorParameters resistor_parameters;
        resistor_parameters.resistance = 1000.0;
        resistor_definition.parameters = resistor_parameters;
        const srp::circuit::ComponentId resistor = circuit.addComponent(resistor_definition);

        const srp::circuit::NodeId first =
            i + 1 < nodes.size() ? nodes[i] : nodes.back();
        const srp::circuit::NodeId second =
            i + 1 < nodes.size() ? nodes[i + 1] : srp::circuit::kGroundNodeId;
        circuit.connectPort(circuit.component(resistor)->ports[0], first);
        circuit.connectPort(circuit.component(resistor)->ports[1], second);
    }

    return circuit;
}

double benchDcSolve(std::size_t node_count, std::size_t repetitions)
{
    const srp::circuit::CircuitModel circuit = buildLadderCircuit(node_count);
    for (std::size_t i = 0; i < 10; ++i)
    {
        const auto result = srp::circuit::solveDc(circuit);
        if (!result.has_value())
        {
            return -1.0;
        }
    }

    const auto start = Clock::now();
    for (std::size_t i = 0; i < repetitions; ++i)
    {
        const auto result = srp::circuit::solveDc(circuit);
        if (!result.has_value())
        {
            return -1.0;
        }
    }
    const auto end = Clock::now();
    return elapsedMilliseconds(start, end) / static_cast<double>(repetitions);
}

double benchTransient(
    std::size_t node_count,
    std::size_t steps,
    std::size_t repetitions)
{
    const srp::circuit::CircuitModel circuit = buildLadderCircuit(node_count);
    srp::circuit::TransientSettings settings;
    settings.time_step = 1e-4;
    settings.end_time = settings.time_step * static_cast<double>(steps);

    const auto start = Clock::now();
    for (std::size_t i = 0; i < repetitions; ++i)
    {
        const auto result = srp::circuit::simulateTransient(circuit, settings);
        if (!result.has_value())
        {
            return -1.0;
        }
    }
    const auto end = Clock::now();
    return elapsedMilliseconds(start, end) /
        static_cast<double>(repetitions * steps);
}

double benchDigital(std::size_t gate_count, std::size_t steps)
{
    srp::circuit::CircuitModel circuit;

    srp::circuit::ComponentDefinition source_definition;
    source_definition.type = srp::circuit::ComponentType::kDigitalSource;
    source_definition.name = "CLK";
    source_definition.port_names = {"output"};
    srp::circuit::DigitalSourceParameters source_parameters;
    source_parameters.frequency_hz = 100.0;
    source_definition.parameters = source_parameters;
    const srp::circuit::ComponentId source = circuit.addComponent(source_definition);
    const srp::circuit::NodeId clock_node = circuit.addNode("clock");
    circuit.connectPort(circuit.component(source)->ports[0], clock_node);

    srp::circuit::NodeId previous = clock_node;
    for (std::size_t i = 0; i < gate_count; ++i)
    {
        srp::circuit::ComponentDefinition gate_definition;
        gate_definition.type = srp::circuit::ComponentType::kLogicGate;
        gate_definition.name = "G" + std::to_string(i);
        gate_definition.port_names = {"input_a", "input_b", "output"};
        srp::circuit::LogicGateParameters gate_parameters;
        gate_parameters.type = srp::circuit::LogicGateType::kAnd;
        gate_definition.parameters = gate_parameters;
        const srp::circuit::ComponentId gate = circuit.addComponent(gate_definition);

        const srp::circuit::NodeId output = circuit.addNode("g" + std::to_string(i));
        circuit.connectPort(circuit.component(gate)->ports[0], previous);
        circuit.connectPort(circuit.component(gate)->ports[1], clock_node);
        circuit.connectPort(circuit.component(gate)->ports[2], output);
        previous = output;
    }

    srp::circuit::DigitalSettings settings;
    settings.time_step = 1e-3;
    settings.end_time = settings.time_step * static_cast<double>(steps);

    const auto start = Clock::now();
    const auto result = srp::circuit::simulateDigital(circuit, settings);
    const auto end = Clock::now();
    if (!result.has_value())
    {
        return -1.0;
    }
    return elapsedMilliseconds(start, end) / static_cast<double>(steps);
}

}  // namespace

int main()
{
    nlohmann::json report;
    report["tool"] = "srp_bench";

    std::cout << "SRPlatform performance benchmark\n";
    std::cout << std::left << std::setw(28) << "scenario"
              << std::right << std::setw(10) << "units"
              << std::setw(18) << "ms/op"
              << std::setw(14) << "ops/sec"
              << "\n";

    const std::size_t physics_sizes[] = {32, 64, 128};
    for (const std::size_t bodies : physics_sizes)
    {
        const double ms_per_step = benchPhysics(bodies, 120);
        printRow("physics " + std::to_string(bodies) + " bodies", bodies, ms_per_step);
        report["physics"][std::to_string(bodies) + "_bodies"] = {
            {"ms_per_step", ms_per_step},
            {"steps_per_sec", ms_per_step > 0.0 ? 1000.0 / ms_per_step : 0.0}};
    }

    const std::size_t circuit_sizes[] = {16, 32, 64};
    for (const std::size_t nodes : circuit_sizes)
    {
        const double ms_per_solve = benchDcSolve(nodes, 50);
        printRow("dc solve " + std::to_string(nodes) + " nodes", nodes, ms_per_solve);
        report["circuit_dc"][std::to_string(nodes) + "_nodes"] = {
            {"ms_per_solve", ms_per_solve},
            {"solves_per_sec", ms_per_solve > 0.0 ? 1000.0 / ms_per_solve : 0.0}};
    }

    for (const std::size_t nodes : circuit_sizes)
    {
        const double ms_per_step = benchTransient(nodes, 2000, 5);
        printRow("transient " + std::to_string(nodes) + " nodes", nodes, ms_per_step);
        report["circuit_transient"][std::to_string(nodes) + "_nodes"] = {
            {"ms_per_step", ms_per_step}};
    }

    const double digital_ms = benchDigital(16, 5000);
    printRow("digital 16 gates", 16, digital_ms);
    report["circuit_digital"]["16_gates"] = {
        {"ms_per_step", digital_ms}};

    std::ofstream stream("logs/benchmark.json");
    if (stream.is_open())
    {
        stream << report.dump(2);
        std::cout << "\nwrote logs/benchmark.json\n";
    }

    return 0;
}
