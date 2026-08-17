#include "circuit/netlist_loader.hpp"

#include <cassert>
#include <fstream>
#include <unordered_map>

namespace srp::circuit
{
namespace
{

std::optional<LogicGateType> parseLogicGateType(const std::string& type)
{
    if (type == "not")
    {
        return LogicGateType::kNot;
    }
    if (type == "and")
    {
        return LogicGateType::kAnd;
    }
    if (type == "or")
    {
        return LogicGateType::kOr;
    }
    if (type == "nand")
    {
        return LogicGateType::kNand;
    }
    if (type == "nor")
    {
        return LogicGateType::kNor;
    }
    if (type == "xor")
    {
        return LogicGateType::kXor;
    }
    if (type == "xnor")
    {
        return LogicGateType::kXnor;
    }
    return std::nullopt;
}

bool readNumber(
    const nlohmann::json& parameters,
    const char* key,
    double& output,
    std::string& error)
{
    const auto it = parameters.find(key);
    if (it == parameters.end() || it->is_null())
    {
        return true;
    }
    if (!it->is_number())
    {
        error = std::string("parameter '") + key + "' must be a number";
        return false;
    }
    output = it->get<double>();
    return true;
}

bool readBool(
    const nlohmann::json& parameters,
    const char* key,
    bool& output,
    std::string& error)
{
    const auto it = parameters.find(key);
    if (it == parameters.end() || it->is_null())
    {
        return true;
    }
    if (!it->is_boolean())
    {
        error = std::string("parameter '") + key + "' must be a boolean";
        return false;
    }
    output = it->get<bool>();
    return true;
}

bool applyParameters(
    ComponentType type,
    const nlohmann::json& parameters,
    ComponentParameters& base,
    std::string& error)
{
    if (!parameters.is_object())
    {
        error = "component 'parameters' must be an object";
        return false;
    }

    switch (type)
    {
    case ComponentType::kResistor:
    {
        ResistorParameters& params = std::get<ResistorParameters>(base);
        return readNumber(parameters, "resistance", params.resistance, error);
    }
    case ComponentType::kCapacitor:
    {
        CapacitorParameters& params = std::get<CapacitorParameters>(base);
        return readNumber(parameters, "capacitance", params.capacitance, error) &&
               readNumber(parameters, "initial_voltage", params.initial_voltage, error);
    }
    case ComponentType::kInductor:
    {
        InductorParameters& params = std::get<InductorParameters>(base);
        return readNumber(parameters, "inductance", params.inductance, error) &&
               readNumber(parameters, "initial_current", params.initial_current, error);
    }
    case ComponentType::kVoltageSource:
    {
        VoltageSourceParameters& params =
            std::get<VoltageSourceParameters>(base);
        return readNumber(parameters, "voltage", params.voltage, error);
    }
    case ComponentType::kCurrentSource:
    {
        CurrentSourceParameters& params =
            std::get<CurrentSourceParameters>(base);
        return readNumber(parameters, "current", params.current, error);
    }
    case ComponentType::kDiode:
    {
        DiodeParameters& params = std::get<DiodeParameters>(base);
        return readNumber(parameters, "forward_voltage", params.forward_voltage, error) &&
               readNumber(parameters, "on_resistance", params.on_resistance, error) &&
               readNumber(parameters, "off_resistance", params.off_resistance, error);
    }
    case ComponentType::kSwitch:
    {
        SwitchParameters& params = std::get<SwitchParameters>(base);
        return readBool(parameters, "closed", params.closed, error) &&
               readNumber(parameters, "on_resistance", params.on_resistance, error) &&
               readNumber(parameters, "off_resistance", params.off_resistance, error);
    }
    case ComponentType::kDigitalSource:
    {
        DigitalSourceParameters& params =
            std::get<DigitalSourceParameters>(base);
        return readBool(parameters, "initial_value", params.initial_value, error) &&
               readNumber(parameters, "frequency_hz", params.frequency_hz, error);
    }
    case ComponentType::kLogicGate:
    {
        LogicGateParameters& params = std::get<LogicGateParameters>(base);
        const auto type_it = parameters.find("type");
        if (type_it == parameters.end() || type_it->is_null())
        {
            return true;
        }
        if (!type_it->is_string())
        {
            error = "logic_gate 'type' parameter must be a string";
            return false;
        }
        const std::optional<LogicGateType> gate_type =
            parseLogicGateType(type_it->get<std::string>());
        if (!gate_type.has_value())
        {
            error = "unknown logic gate type: " + type_it->get<std::string>();
            return false;
        }
        params.type = *gate_type;
        return true;
    }
    case ComponentType::kDFlipFlop:
    {
        DFlipFlopParameters& params =
            std::get<DFlipFlopParameters>(base);
        return readBool(parameters, "initial_q", params.initial_q, error);
    }
    case ComponentType::kPwmSource:
    {
        PwmSourceParameters& params =
            std::get<PwmSourceParameters>(base);
        return readNumber(parameters, "frequency_hz", params.frequency_hz, error) &&
               readNumber(parameters, "duty_cycle", params.duty_cycle, error);
    }
    }

    assert(false && "unreachable component type");
    return false;
}

bool isGroundName(const std::string& name)
{
    return name == "gnd" || name == "ground" || name == "0";
}

}  // namespace

std::optional<CircuitModel> loadNetlist(
    const nlohmann::json& json,
    std::string& error)
{
    return loadNetlist(json, ComponentLibrary{}, error);
}

std::optional<CircuitModel> loadNetlist(
    const nlohmann::json& json,
    const ComponentLibrary& library,
    std::string& error)
{
    error.clear();

    if (!json.is_object())
    {
        error = "netlist must be a JSON object";
        return std::nullopt;
    }

    CircuitModel circuit;
    std::unordered_map<std::string, NodeId> node_ids;
    node_ids["gnd"] = kGroundNodeId;
    node_ids["ground"] = kGroundNodeId;
    node_ids["0"] = kGroundNodeId;

    if (const auto nodes_it = json.find("nodes");
        nodes_it != json.end() && !nodes_it->is_null())
    {
        if (!nodes_it->is_array())
        {
            error = "netlist 'nodes' must be an array of names";
            return std::nullopt;
        }

        for (const nlohmann::json& entry : *nodes_it)
        {
            if (!entry.is_string())
            {
                error = "netlist 'nodes' entries must be strings";
                return std::nullopt;
            }

            const std::string name = entry.get<std::string>();
            if (isGroundName(name))
            {
                continue;
            }
            if (node_ids.find(name) != node_ids.end())
            {
                error = "duplicate node name: " + name;
                return std::nullopt;
            }
            node_ids[name] = circuit.addNode(name);
        }
    }

    const auto components_it = json.find("components");
    if (components_it == json.end() || !components_it->is_array())
    {
        error = "netlist requires a 'components' array";
        return std::nullopt;
    }

    for (std::size_t index = 0; index < components_it->size(); ++index)
    {
        const nlohmann::json& entry = (*components_it)[index];
        if (!entry.is_object())
        {
            error = "netlist component entries must be objects";
            return std::nullopt;
        }

        const auto type_it = entry.find("type");
        if (type_it == entry.end() || !type_it->is_string())
        {
            error = "netlist component requires a string 'type'";
            return std::nullopt;
        }
        const std::string type_name = type_it->get<std::string>();
        const std::optional<ComponentDefinition> template_definition =
            library.find(type_name);
        if (!template_definition.has_value())
        {
            error = "unknown component type: " + type_name;
            return std::nullopt;
        }

        ComponentDefinition definition = *template_definition;
        if (const auto name_it = entry.find("name");
            name_it != entry.end() && name_it->is_string())
        {
            definition.name = name_it->get<std::string>();
        }

        if (const auto parameters_it = entry.find("parameters");
            parameters_it != entry.end() && !parameters_it->is_null())
        {
            if (!applyParameters(
                    definition.type,
                    *parameters_it,
                    definition.parameters,
                    error))
            {
                return std::nullopt;
            }
        }

        const auto ports_it = entry.find("ports");
        if (ports_it == entry.end() || !ports_it->is_array())
        {
            error = "netlist component requires a 'ports' array of node names";
            return std::nullopt;
        }
        const std::size_t expected_ports =
            defaultPortCount(definition.type);
        if (ports_it->size() != expected_ports)
        {
            error = "component '" + definition.name +
                    "' expects " + std::to_string(expected_ports) +
                    " ports but got " + std::to_string(ports_it->size());
            return std::nullopt;
        }

        const ComponentId component_id =
            circuit.addComponent(definition);
        const Component* component = circuit.component(component_id);
        if (component == nullptr)
        {
            error = "failed to create component '" + definition.name + "'";
            return std::nullopt;
        }

        for (std::size_t port_index = 0; port_index < ports_it->size(); ++port_index)
        {
            const nlohmann::json& port_entry = (*ports_it)[port_index];
            if (!port_entry.is_string())
            {
                error = "netlist component port entries must be strings";
                return std::nullopt;
            }

            const std::string node_name = port_entry.get<std::string>();
            if (node_name.empty())
            {
                // Empty port entries describe intentionally unwired ports.
                continue;
            }
            const auto node_it = node_ids.find(node_name);
            NodeId node_id;
            if (node_it == node_ids.end())
            {
                if (isGroundName(node_name))
                {
                    node_id = kGroundNodeId;
                }
                else
                {
                    node_id = circuit.addNode(node_name);
                    node_ids[node_name] = node_id;
                }
            }
            else
            {
                node_id = node_it->second;
            }

            if (!circuit.connectPort(component->ports[port_index], node_id))
            {
                error = "failed to connect component '" +
                        definition.name + "' port " +
                        std::to_string(port_index);
                return std::nullopt;
            }
        }
    }

    return circuit;
}

std::optional<CircuitModel> loadNetlistFile(
    const std::filesystem::path& path,
    std::string& error)
{
    error.clear();

    std::ifstream stream(path);
    if (!stream.is_open())
    {
        error = "cannot open netlist file: " + path.string();
        return std::nullopt;
    }

    nlohmann::json json;
    try
    {
        stream >> json;
    }
    catch (const nlohmann::json::parse_error& parse_error)
    {
        error = std::string("invalid JSON in netlist file: ") + parse_error.what();
        return std::nullopt;
    }

    return loadNetlist(json, error);
}

}  // namespace srp::circuit
