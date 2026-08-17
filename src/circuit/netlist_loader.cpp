#include "circuit/netlist_loader.hpp"

#include <fstream>
#include <unordered_map>

namespace srp::circuit
{
namespace
{

std::optional<ComponentType> parseComponentType(const std::string& type)
{
    if (type == "resistor")
    {
        return ComponentType::kResistor;
    }
    if (type == "capacitor")
    {
        return ComponentType::kCapacitor;
    }
    if (type == "inductor")
    {
        return ComponentType::kInductor;
    }
    if (type == "voltage_source")
    {
        return ComponentType::kVoltageSource;
    }
    if (type == "current_source")
    {
        return ComponentType::kCurrentSource;
    }
    if (type == "diode")
    {
        return ComponentType::kDiode;
    }
    if (type == "switch")
    {
        return ComponentType::kSwitch;
    }
    if (type == "digital_source")
    {
        return ComponentType::kDigitalSource;
    }
    if (type == "logic_gate")
    {
        return ComponentType::kLogicGate;
    }
    if (type == "d_flip_flop")
    {
        return ComponentType::kDFlipFlop;
    }
    if (type == "pwm_source")
    {
        return ComponentType::kPwmSource;
    }
    return std::nullopt;
}

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

bool parseParameters(
    ComponentType type,
    const nlohmann::json& parameters,
    ComponentParameters& output,
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
        ResistorParameters params;
        if (!readNumber(parameters, "resistance", params.resistance, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kCapacitor:
    {
        CapacitorParameters params;
        if (!readNumber(parameters, "capacitance", params.capacitance, error) ||
            !readNumber(parameters, "initial_voltage", params.initial_voltage, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kInductor:
    {
        InductorParameters params;
        if (!readNumber(parameters, "inductance", params.inductance, error) ||
            !readNumber(parameters, "initial_current", params.initial_current, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kVoltageSource:
    {
        VoltageSourceParameters params;
        if (!readNumber(parameters, "voltage", params.voltage, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kCurrentSource:
    {
        CurrentSourceParameters params;
        if (!readNumber(parameters, "current", params.current, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kDiode:
    {
        DiodeParameters params;
        if (!readNumber(parameters, "forward_voltage", params.forward_voltage, error) ||
            !readNumber(parameters, "on_resistance", params.on_resistance, error) ||
            !readNumber(parameters, "off_resistance", params.off_resistance, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kSwitch:
    {
        SwitchParameters params;
        if (!readBool(parameters, "closed", params.closed, error) ||
            !readNumber(parameters, "on_resistance", params.on_resistance, error) ||
            !readNumber(parameters, "off_resistance", params.off_resistance, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kDigitalSource:
    {
        DigitalSourceParameters params;
        if (!readBool(parameters, "initial_value", params.initial_value, error) ||
            !readNumber(parameters, "frequency_hz", params.frequency_hz, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kLogicGate:
    {
        LogicGateParameters params;
        const auto type_it = parameters.find("type");
        if (type_it == parameters.end() || !type_it->is_string())
        {
            error = "logic_gate requires a string 'type' parameter";
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
        output = params;
        return true;
    }
    case ComponentType::kDFlipFlop:
    {
        DFlipFlopParameters params;
        if (!readBool(parameters, "initial_q", params.initial_q, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    case ComponentType::kPwmSource:
    {
        PwmSourceParameters params;
        if (!readNumber(parameters, "frequency_hz", params.frequency_hz, error) ||
            !readNumber(parameters, "duty_cycle", params.duty_cycle, error))
        {
            return false;
        }
        output = params;
        return true;
    }
    }

    error = "unsupported component type";
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
        const std::optional<ComponentType> type =
            parseComponentType(type_it->get<std::string>());
        if (!type.has_value())
        {
            error = "unknown component type: " + type_it->get<std::string>();
            return std::nullopt;
        }

        ComponentDefinition definition;
        definition.type = *type;
        definition.port_names = defaultPortNames(*type);
        if (const auto name_it = entry.find("name");
            name_it != entry.end() && name_it->is_string())
        {
            definition.name = name_it->get<std::string>();
        }

        const nlohmann::json empty_parameters = nlohmann::json::object();
        const nlohmann::json& parameters =
            entry.value("parameters", empty_parameters);
        if (!parseParameters(*type, parameters, definition.parameters, error))
        {
            return std::nullopt;
        }

        const auto ports_it = entry.find("ports");
        if (ports_it == entry.end() || !ports_it->is_array())
        {
            error = "netlist component requires a 'ports' array of node names";
            return std::nullopt;
        }
        const std::size_t expected_ports = defaultPortCount(*type);
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
