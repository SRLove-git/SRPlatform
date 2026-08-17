#include "circuit/component_library.hpp"

namespace srp::circuit
{

std::optional<ComponentType> builtinComponentType(const std::string& name)
{
    if (name == "resistor")
    {
        return ComponentType::kResistor;
    }
    if (name == "capacitor")
    {
        return ComponentType::kCapacitor;
    }
    if (name == "inductor")
    {
        return ComponentType::kInductor;
    }
    if (name == "voltage_source")
    {
        return ComponentType::kVoltageSource;
    }
    if (name == "current_source")
    {
        return ComponentType::kCurrentSource;
    }
    if (name == "diode")
    {
        return ComponentType::kDiode;
    }
    if (name == "switch")
    {
        return ComponentType::kSwitch;
    }
    if (name == "digital_source")
    {
        return ComponentType::kDigitalSource;
    }
    if (name == "logic_gate")
    {
        return ComponentType::kLogicGate;
    }
    if (name == "d_flip_flop")
    {
        return ComponentType::kDFlipFlop;
    }
    if (name == "pwm_source")
    {
        return ComponentType::kPwmSource;
    }
    return std::nullopt;
}

bool ComponentLibrary::registerComponent(
    const std::string& name,
    const ComponentDefinition& definition)
{
    if (name.empty() ||
        contains(name) ||
        !matchesType(definition.type, definition.parameters))
    {
        return false;
    }

    custom_[name] = definition;
    return true;
}

bool ComponentLibrary::unregisterComponent(const std::string& name)
{
    return custom_.erase(name) > 0;
}

bool ComponentLibrary::contains(const std::string& name) const
{
    if (custom_.find(name) != custom_.end())
    {
        return true;
    }
    return builtinComponentType(name).has_value();
}

std::size_t ComponentLibrary::size() const
{
    return custom_.size();
}

std::vector<std::string> ComponentLibrary::names() const
{
    std::vector<std::string> result;
    result.reserve(custom_.size());
    for (const auto& [name, definition] : custom_)
    {
        (void)definition;
        result.push_back(name);
    }
    return result;
}

std::optional<ComponentDefinition> ComponentLibrary::find(
    const std::string& name) const
{
    const auto custom_it = custom_.find(name);
    if (custom_it != custom_.end())
    {
        return custom_it->second;
    }

    const std::optional<ComponentType> type = builtinComponentType(name);
    if (!type.has_value())
    {
        return std::nullopt;
    }

    ComponentDefinition definition;
    definition.type = *type;
    definition.name = name;
    definition.port_names = defaultPortNames(*type);

    switch (*type)
    {
    case ComponentType::kResistor:
        definition.parameters = ResistorParameters{};
        break;
    case ComponentType::kCapacitor:
        definition.parameters = CapacitorParameters{};
        break;
    case ComponentType::kInductor:
        definition.parameters = InductorParameters{};
        break;
    case ComponentType::kVoltageSource:
        definition.parameters = VoltageSourceParameters{};
        break;
    case ComponentType::kCurrentSource:
        definition.parameters = CurrentSourceParameters{};
        break;
    case ComponentType::kDiode:
        definition.parameters = DiodeParameters{};
        break;
    case ComponentType::kSwitch:
        definition.parameters = SwitchParameters{};
        break;
    case ComponentType::kDigitalSource:
        definition.parameters = DigitalSourceParameters{};
        break;
    case ComponentType::kLogicGate:
        definition.parameters = LogicGateParameters{};
        break;
    case ComponentType::kDFlipFlop:
        definition.parameters = DFlipFlopParameters{};
        break;
    case ComponentType::kPwmSource:
        definition.parameters = PwmSourceParameters{};
        break;
    }

    return definition;
}

std::optional<ComponentId> ComponentLibrary::instantiate(
    CircuitModel& circuit,
    const std::string& name) const
{
    const std::optional<ComponentDefinition> definition = find(name);
    if (!definition.has_value())
    {
        return std::nullopt;
    }

    const ComponentId id = circuit.addComponent(*definition);
    if (id == kInvalidComponentId)
    {
        return std::nullopt;
    }
    return id;
}

}  // namespace srp::circuit
