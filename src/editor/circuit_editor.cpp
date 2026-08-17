#include "editor/circuit_editor.hpp"

#include "circuit/netlist_loader.hpp"

#include <type_traits>
#include <utility>

namespace srp::editor
{
namespace
{

srp::circuit::ComponentParameters defaultParameters(srp::circuit::ComponentType type)
{
    switch (type)
    {
    case srp::circuit::ComponentType::kResistor:
    {
        srp::circuit::ResistorParameters parameters;
        parameters.resistance = 1000.0;
        return parameters;
    }
    case srp::circuit::ComponentType::kCapacitor:
    {
        srp::circuit::CapacitorParameters parameters;
        parameters.capacitance = 1e-3;
        return parameters;
    }
    case srp::circuit::ComponentType::kInductor:
    {
        srp::circuit::InductorParameters parameters;
        parameters.inductance = 0.1;
        return parameters;
    }
    case srp::circuit::ComponentType::kVoltageSource:
    {
        srp::circuit::VoltageSourceParameters parameters;
        parameters.voltage = 5.0;
        return parameters;
    }
    case srp::circuit::ComponentType::kCurrentSource:
    {
        srp::circuit::CurrentSourceParameters parameters;
        parameters.current = 0.01;
        return parameters;
    }
    case srp::circuit::ComponentType::kDiode:
        return srp::circuit::DiodeParameters{};
    case srp::circuit::ComponentType::kSwitch:
        return srp::circuit::SwitchParameters{};
    case srp::circuit::ComponentType::kDigitalSource:
        return srp::circuit::DigitalSourceParameters{};
    case srp::circuit::ComponentType::kLogicGate:
    {
        srp::circuit::LogicGateParameters parameters;
        parameters.type = srp::circuit::LogicGateType::kAnd;
        return parameters;
    }
    case srp::circuit::ComponentType::kDFlipFlop:
        return srp::circuit::DFlipFlopParameters{};
    case srp::circuit::ComponentType::kPwmSource:
    {
        srp::circuit::PwmSourceParameters parameters;
        parameters.frequency_hz = 1000.0;
        parameters.duty_cycle = 0.5;
        return parameters;
    }
    }
    return srp::circuit::ResistorParameters{};
}

const char* netlistTypeName(srp::circuit::ComponentType type)
{
    switch (type)
    {
    case srp::circuit::ComponentType::kResistor:
        return "resistor";
    case srp::circuit::ComponentType::kCapacitor:
        return "capacitor";
    case srp::circuit::ComponentType::kInductor:
        return "inductor";
    case srp::circuit::ComponentType::kVoltageSource:
        return "voltage_source";
    case srp::circuit::ComponentType::kCurrentSource:
        return "current_source";
    case srp::circuit::ComponentType::kDiode:
        return "diode";
    case srp::circuit::ComponentType::kSwitch:
        return "switch";
    case srp::circuit::ComponentType::kDigitalSource:
        return "digital_source";
    case srp::circuit::ComponentType::kLogicGate:
        return "logic_gate";
    case srp::circuit::ComponentType::kDFlipFlop:
        return "d_flip_flop";
    case srp::circuit::ComponentType::kPwmSource:
        return "pwm_source";
    }
    return "resistor";
}

const char* logicGateTypeName(srp::circuit::LogicGateType type)
{
    switch (type)
    {
    case srp::circuit::LogicGateType::kNot:
        return "not";
    case srp::circuit::LogicGateType::kAnd:
        return "and";
    case srp::circuit::LogicGateType::kOr:
        return "or";
    case srp::circuit::LogicGateType::kNand:
        return "nand";
    case srp::circuit::LogicGateType::kNor:
        return "nor";
    case srp::circuit::LogicGateType::kXor:
        return "xor";
    case srp::circuit::LogicGateType::kXnor:
        return "xnor";
    }
    return "and";
}

nlohmann::json parametersToJson(const srp::circuit::ComponentParameters& parameters)
{
    return std::visit(
        [](const auto& value) -> nlohmann::json
        {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, srp::circuit::ResistorParameters>)
            {
                return nlohmann::json{{"resistance", value.resistance}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::CapacitorParameters>)
            {
                return nlohmann::json{
                    {"capacitance", value.capacitance},
                    {"initial_voltage", value.initial_voltage}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::InductorParameters>)
            {
                return nlohmann::json{
                    {"inductance", value.inductance},
                    {"initial_current", value.initial_current}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::VoltageSourceParameters>)
            {
                return nlohmann::json{{"voltage", value.voltage}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::CurrentSourceParameters>)
            {
                return nlohmann::json{{"current", value.current}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::DiodeParameters>)
            {
                return nlohmann::json{
                    {"forward_voltage", value.forward_voltage},
                    {"on_resistance", value.on_resistance},
                    {"off_resistance", value.off_resistance}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::SwitchParameters>)
            {
                return nlohmann::json{
                    {"closed", value.closed},
                    {"on_resistance", value.on_resistance},
                    {"off_resistance", value.off_resistance}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::DigitalSourceParameters>)
            {
                return nlohmann::json{
                    {"initial_value", value.initial_value},
                    {"frequency_hz", value.frequency_hz}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::LogicGateParameters>)
            {
                return nlohmann::json{{"type", logicGateTypeName(value.type)}};
            }
            else if constexpr (std::is_same_v<T, srp::circuit::DFlipFlopParameters>)
            {
                return nlohmann::json{{"initial_q", value.initial_q}};
            }
            else
            {
                return nlohmann::json{
                    {"frequency_hz", value.frequency_hz},
                    {"duty_cycle", value.duty_cycle}};
            }
        },
        parameters);
}

}  // namespace

CircuitEditor::CircuitEditor() = default;

srp::circuit::ComponentId CircuitEditor::addComponent(
    srp::circuit::ComponentType type,
    const srp::math::Vec2& position)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = type;
    definition.name = std::string(srp::circuit::componentTypeName(type));
    definition.port_names = srp::circuit::defaultPortNames(type);
    definition.parameters = defaultParameters(type);

    const srp::circuit::ComponentId id = circuit_.addComponent(definition);
    if (id == srp::circuit::kInvalidComponentId)
    {
        return srp::circuit::kInvalidComponentId;
    }

    positions_[id] = position;
    return id;
}

bool CircuitEditor::removeComponent(srp::circuit::ComponentId id)
{
    if (selected_ == id)
    {
        selected_.reset();
    }
    if (circuit_.component(id) == nullptr)
    {
        return false;
    }
    positions_.erase(id);
    return circuit_.removeComponent(id);
}

void CircuitEditor::clear()
{
    const std::vector<srp::circuit::Component> components = circuit_.components();
    for (const srp::circuit::Component& component : components)
    {
        removeComponent(component.id);
    }
    selected_.reset();
    pending_wire_port_.reset();
}

bool CircuitEditor::wire(
    srp::circuit::PortId port_a,
    srp::circuit::PortId port_b)
{
    if (port_a == port_b)
    {
        return false;
    }

    srp::circuit::Port* port_a_ptr = circuit_.port(port_a);
    srp::circuit::Port* port_b_ptr = circuit_.port(port_b);
    if (port_a_ptr == nullptr || port_b_ptr == nullptr)
    {
        return false;
    }

    if (port_a_ptr->node == port_b_ptr->node &&
        port_a_ptr->node != srp::circuit::kInvalidNodeId)
    {
        return true;
    }

    srp::circuit::NodeId target_node = port_a_ptr->node;
    if (target_node == srp::circuit::kInvalidNodeId)
    {
        target_node = port_b_ptr->node;
    }
    if (target_node == srp::circuit::kInvalidNodeId)
    {
        target_node = circuit_.addNode();
    }

    return circuit_.connectPort(port_a, target_node) &&
           circuit_.connectPort(port_b, target_node);
}

bool CircuitEditor::unwire(srp::circuit::PortId port_id)
{
    if (!circuit_.disconnectPort(port_id))
    {
        return false;
    }
    circuit_.pruneEmptyNodes();
    return true;
}

srp::circuit::CircuitModel& CircuitEditor::circuit()
{
    return circuit_;
}

const srp::circuit::CircuitModel& CircuitEditor::circuit() const
{
    return circuit_;
}

srp::math::Vec2 CircuitEditor::componentPosition(srp::circuit::ComponentId id) const
{
    const auto it = positions_.find(id);
    return it == positions_.end() ? srp::math::Vec2(0.0) : it->second;
}

bool CircuitEditor::setComponentPosition(
    srp::circuit::ComponentId id,
    const srp::math::Vec2& position)
{
    const auto it = positions_.find(id);
    if (it == positions_.end())
    {
        return false;
    }
    it->second = position;
    return true;
}

srp::math::Vec2 CircuitEditor::portPosition(srp::circuit::PortId id) const
{
    const srp::circuit::Port* port = circuit_.port(id);
    if (port == nullptr)
    {
        return srp::math::Vec2(0.0);
    }

    const srp::circuit::Component* component = circuit_.component(port->component);
    if (component == nullptr)
    {
        return srp::math::Vec2(0.0);
    }

    std::size_t port_index = 0;
    for (; port_index < component->ports.size(); ++port_index)
    {
        if (component->ports[port_index] == id)
        {
            break;
        }
    }

    return componentPosition(component->id) +
           portOffset(component->definition, port_index);
}

srp::math::Vec2 CircuitEditor::portOffset(
    const srp::circuit::ComponentDefinition& definition,
    std::size_t port_index)
{
    const std::size_t count = definition.port_names.size();
    switch (count)
    {
    case 1:
        return srp::math::Vec2(0.9, 0.0);
    case 2:
        return port_index == 0
            ? srp::math::Vec2(-0.9, 0.0)
            : srp::math::Vec2(0.9, 0.0);
    case 3:
        if (port_index == 0)
        {
            return srp::math::Vec2(-0.9, -0.35);
        }
        if (port_index == 1)
        {
            return srp::math::Vec2(-0.9, 0.35);
        }
        return srp::math::Vec2(0.9, 0.0);
    case 4:
        if (port_index == 0)
        {
            return srp::math::Vec2(-0.9, -0.3);
        }
        if (port_index == 1)
        {
            return srp::math::Vec2(-0.9, 0.3);
        }
        if (port_index == 2)
        {
            return srp::math::Vec2(0.9, -0.3);
        }
        return srp::math::Vec2(0.9, 0.3);
    }
    return srp::math::Vec2(0.0);
}

bool CircuitEditor::select(srp::circuit::ComponentId id)
{
    if (circuit_.component(id) == nullptr)
    {
        return false;
    }
    selected_ = id;
    return true;
}

void CircuitEditor::deselect()
{
    selected_.reset();
}

std::optional<srp::circuit::ComponentId> CircuitEditor::selected() const
{
    return selected_;
}

void CircuitEditor::setPendingWirePort(srp::circuit::PortId id)
{
    if (circuit_.port(id) != nullptr)
    {
        pending_wire_port_ = id;
    }
}

std::optional<srp::circuit::PortId> CircuitEditor::pendingWirePort() const
{
    return pending_wire_port_;
}

void CircuitEditor::cancelWire()
{
    pending_wire_port_.reset();
}

nlohmann::json CircuitEditor::toNetlistJson() const
{
    nlohmann::json json;
    json["version"] = 1;

    nlohmann::json component_array = nlohmann::json::array();
    for (const srp::circuit::Component& component : circuit_.components())
    {
        nlohmann::json component_json;
        component_json["type"] = netlistTypeName(component.definition.type);
        if (!component.definition.name.empty())
        {
            component_json["name"] = component.definition.name;
        }

        nlohmann::json port_array = nlohmann::json::array();
        for (const srp::circuit::PortId port_id : component.ports)
        {
            const srp::circuit::Port* port = circuit_.port(port_id);
            if (port == nullptr || port->node == srp::circuit::kInvalidNodeId)
            {
                port_array.push_back("");
                continue;
            }

            const srp::circuit::Node* node = circuit_.node(port->node);
            port_array.push_back(
                node != nullptr ? node->name : "n" + std::to_string(port->node));
        }
        component_json["ports"] = std::move(port_array);
        component_json["parameters"] =
            parametersToJson(component.definition.parameters);
        component_array.push_back(std::move(component_json));
    }

    json["components"] = std::move(component_array);
    return json;
}

bool CircuitEditor::loadNetlistJson(
    const nlohmann::json& json,
    std::string& error)
{
    auto loaded = srp::circuit::loadNetlist(json, error);
    if (!loaded.has_value())
    {
        return false;
    }

    clear();
    circuit_ = std::move(*loaded);

    std::size_t index = 0;
    for (const srp::circuit::Component& component : circuit_.components())
    {
        const double x = static_cast<double>(index % 4) * 3.0;
        const double y = static_cast<double>(index / 4) * 2.5;
        positions_[component.id] = srp::math::Vec2(x, y);
        ++index;
    }
    return true;
}

}  // namespace srp::editor
