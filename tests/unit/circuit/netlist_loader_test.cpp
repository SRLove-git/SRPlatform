#include "circuit/dc_solver.hpp"
#include "circuit/netlist_loader.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <variant>

namespace
{

nlohmann::json voltageDividerNetlist()
{
    nlohmann::json netlist;
    netlist["nodes"] = nlohmann::json::array({"vcc", "out"});
    netlist["components"] = nlohmann::json::array();

    nlohmann::json source;
    source["type"] = "voltage_source";
    source["name"] = "V1";
    source["ports"] = nlohmann::json::array({"vcc", "gnd"});
    source["parameters"]["voltage"] = 12.0;
    netlist["components"].push_back(source);

    nlohmann::json top;
    top["type"] = "resistor";
    top["name"] = "R1";
    top["ports"] = nlohmann::json::array({"vcc", "out"});
    top["parameters"]["resistance"] = 1000.0;
    netlist["components"].push_back(top);

    nlohmann::json bottom;
    bottom["type"] = "resistor";
    bottom["name"] = "R2";
    bottom["ports"] = nlohmann::json::array({"out", "gnd"});
    bottom["parameters"]["resistance"] = 2000.0;
    netlist["components"].push_back(bottom);

    return netlist;
}

srp::circuit::NodeId findNode(
    const srp::circuit::CircuitModel& circuit,
    const std::string& name)
{
    for (const srp::circuit::Node& node : circuit.nodes())
    {
        if (node.name == name)
        {
            return node.id;
        }
    }
    return srp::circuit::kInvalidNodeId;
}

std::filesystem::path writeTempFile(
    const std::string& name,
    const std::string& content)
{
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_netlist_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path path = directory / name;
    std::ofstream(path) << content;
    return path;
}

}  // namespace

TEST(NetlistLoader, LoadsVoltageDividerAndSolves)
{
    std::string error;
    const auto circuit =
        srp::circuit::loadNetlist(voltageDividerNetlist(), error);

    ASSERT_TRUE(circuit.has_value()) << error;
    ASSERT_EQ(circuit->components().size(), 3U);

    const srp::circuit::NodeId out = findNode(*circuit, "out");
    ASSERT_NE(out, srp::circuit::kInvalidNodeId);

    const auto result = srp::circuit::solveDc(*circuit);
    ASSERT_TRUE(result.has_value());
    const auto voltage = srp::circuit::nodeVoltage(*result, out);
    ASSERT_TRUE(voltage.has_value());
    EXPECT_NEAR(*voltage, 8.0, 1e-9);

    const srp::circuit::Component* top = circuit->component(2);
    ASSERT_NE(top, nullptr);
    const auto current = srp::circuit::componentCurrent(*result, top->id);
    ASSERT_TRUE(current.has_value());
    EXPECT_NEAR(*current, 4e-3, 1e-12);
}

TEST(NetlistLoader, LoadsEveryComponentType)
{
    nlohmann::json netlist;
    netlist["nodes"] = nlohmann::json::array();
    netlist["components"] = nlohmann::json::array();

    const std::vector<std::pair<std::string, nlohmann::json>> components = {
        {"resistor", {{"resistance", 470.0}}},
        {"capacitor", {{"capacitance", 1e-4}, {"initial_voltage", 1.0}}},
        {"inductor", {{"inductance", 0.01}}},
        {"voltage_source", {{"voltage", 5.0}}},
        {"current_source", {{"current", 0.02}}},
        {"diode", {{"forward_voltage", 0.3}}},
        {"switch", {{"closed", true}}},
        {"digital_source", {{"initial_value", true}}},
        {"logic_gate", {{"type", "nand"}}},
        {"d_flip_flop", {{"initial_q", true}}},
        {"pwm_source", {{"frequency_hz", 50.0}, {"duty_cycle", 0.25}}}};

    for (std::size_t i = 0; i < components.size(); ++i)
    {
        nlohmann::json entry;
        entry["type"] = components[i].first;
        entry["name"] = "C" + std::to_string(i);
        entry["parameters"] = components[i].second;

        std::size_t port_count = 2;
        if (components[i].first == "digital_source" ||
            components[i].first == "pwm_source")
        {
            port_count = 1;
        }
        else if (components[i].first == "logic_gate")
        {
            port_count = 3;
        }
        else if (components[i].first == "d_flip_flop")
        {
            port_count = 4;
        }

        nlohmann::json ports = nlohmann::json::array();
        for (std::size_t p = 0; p < port_count; ++p)
        {
            ports.push_back("gnd");
        }
        entry["ports"] = ports;
        netlist["components"].push_back(entry);
    }

    std::string error;
    const auto circuit = srp::circuit::loadNetlist(netlist, error);

    ASSERT_TRUE(circuit.has_value()) << error;
    ASSERT_EQ(circuit->components().size(), components.size());

    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::ResistorParameters>(
            circuit->components()[0].definition.parameters).resistance,
        470.0);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::CapacitorParameters>(
            circuit->components()[1].definition.parameters).initial_voltage,
        1.0);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::VoltageSourceParameters>(
            circuit->components()[3].definition.parameters).voltage,
        5.0);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::DiodeParameters>(
            circuit->components()[5].definition.parameters).forward_voltage,
        0.3);
    EXPECT_TRUE(
        std::get<srp::circuit::SwitchParameters>(
            circuit->components()[6].definition.parameters).closed);
    EXPECT_EQ(
        std::get<srp::circuit::LogicGateParameters>(
            circuit->components()[8].definition.parameters).type,
        srp::circuit::LogicGateType::kNand);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::PwmSourceParameters>(
            circuit->components()[10].definition.parameters).duty_cycle,
        0.25);
}

TEST(NetlistLoader, CreatesImplicitNodesAndMapsGroundAliases)
{
    nlohmann::json netlist;
    netlist["components"] = nlohmann::json::array();

    nlohmann::json first;
    first["type"] = "resistor";
    first["name"] = "R1";
    first["ports"] = nlohmann::json::array({"gnd", "n1"});
    netlist["components"].push_back(first);

    nlohmann::json second;
    second["type"] = "resistor";
    second["name"] = "R2";
    second["ports"] = nlohmann::json::array({"0", "n2"});
    netlist["components"].push_back(second);

    std::string error;
    const auto circuit = srp::circuit::loadNetlist(netlist, error);

    ASSERT_TRUE(circuit.has_value()) << error;
    EXPECT_NE(findNode(*circuit, "n1"), srp::circuit::kInvalidNodeId);
    EXPECT_NE(findNode(*circuit, "n2"), srp::circuit::kInvalidNodeId);

    const srp::circuit::Component* first_component =
        circuit->component(1);
    ASSERT_NE(first_component, nullptr);
    ASSERT_EQ(first_component->ports.size(), 2U);
    EXPECT_EQ(
        circuit->port(first_component->ports[0])->node,
        srp::circuit::kGroundNodeId);
}

TEST(NetlistLoader, RejectsDuplicateNodeNames)
{
    nlohmann::json netlist = voltageDividerNetlist();
    netlist["nodes"] = nlohmann::json::array({"a", "a"});
    std::string error;

    EXPECT_FALSE(srp::circuit::loadNetlist(netlist, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(NetlistLoader, RejectsUnknownComponentType)
{
    nlohmann::json netlist = voltageDividerNetlist();
    netlist["components"][0]["type"] = "quantum_resistor";
    std::string error;

    EXPECT_FALSE(srp::circuit::loadNetlist(netlist, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(NetlistLoader, RejectsWrongPortCount)
{
    nlohmann::json netlist = voltageDividerNetlist();
    netlist["components"][1]["ports"] =
        nlohmann::json::array({"vcc", "out", "gnd"});
    std::string error;

    EXPECT_FALSE(srp::circuit::loadNetlist(netlist, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(NetlistLoader, RejectsWrongParameterType)
{
    nlohmann::json netlist = voltageDividerNetlist();
    netlist["components"][1]["parameters"]["resistance"] = "one thousand";
    std::string error;

    EXPECT_FALSE(srp::circuit::loadNetlist(netlist, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(NetlistLoader, RejectsMissingPorts)
{
    nlohmann::json netlist = voltageDividerNetlist();
    netlist["components"][1].erase("ports");
    std::string error;

    EXPECT_FALSE(srp::circuit::loadNetlist(netlist, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(NetlistLoader, RejectsUnknownGateType)
{
    nlohmann::json netlist;
    nlohmann::json gate;
    gate["type"] = "logic_gate";
    gate["name"] = "G1";
    gate["ports"] = nlohmann::json::array({"a", "b", "out"});
    gate["parameters"]["type"] = "maybe";
    netlist["components"] = nlohmann::json::array({gate});
    std::string error;

    EXPECT_FALSE(srp::circuit::loadNetlist(netlist, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(NetlistLoader, LoadsNetlistFromFile)
{
    const std::filesystem::path path = writeTempFile(
        "divider.json",
        voltageDividerNetlist().dump());

    std::string error;
    const auto circuit = srp::circuit::loadNetlistFile(path, error);

    ASSERT_TRUE(circuit.has_value()) << error;
    ASSERT_EQ(circuit->components().size(), 3U);
    std::filesystem::remove_all(
        std::filesystem::temp_directory_path() / "srp_netlist_test");
}

TEST(NetlistLoader, ReportsMissingAndInvalidFiles)
{
    const std::filesystem::path missing =
        std::filesystem::temp_directory_path() / "srp_netlist_missing" / "net.json";
    std::string error;
    EXPECT_FALSE(srp::circuit::loadNetlistFile(missing, error).has_value());
    EXPECT_FALSE(error.empty());

    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "srp_netlist_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path invalid = directory / "bad.json";
    std::ofstream(invalid) << "{ not json";
    error.clear();
    EXPECT_FALSE(srp::circuit::loadNetlistFile(invalid, error).has_value());
    EXPECT_FALSE(error.empty());
    std::filesystem::remove_all(directory);
}
