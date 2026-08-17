#pragma once

#include "circuit/circuit_model.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace srp::circuit
{

// Registry of reusable component templates.
//
// Built-in types (resistor, capacitor, ...) are always available. Users can
// register custom templates under new names, e.g. a "white_led" built on the
// diode model with a higher forward voltage. Netlist loading can resolve
// component types through this library.
class ComponentLibrary
{
public:
    ComponentLibrary() = default;

    // Registers a custom component template. Returns false if the name is
    // already taken (by a built-in type or another registered template).
    bool registerComponent(
        const std::string& name,
        const ComponentDefinition& definition);

    bool unregisterComponent(const std::string& name);
    bool contains(const std::string& name) const;
    std::size_t size() const;
    std::vector<std::string> names() const;

    // Looks up a template by name: custom templates first, then built-in
    // types. Returns nullopt when nothing matches.
    std::optional<ComponentDefinition> find(const std::string& name) const;

    // Adds an instance of the named template to a circuit.
    std::optional<ComponentId> instantiate(
        CircuitModel& circuit,
        const std::string& name) const;

private:
    std::unordered_map<std::string, ComponentDefinition> custom_;
};

// Returns the component type for a built-in type name (e.g. "resistor"),
// or nullopt for unknown names.
std::optional<ComponentType> builtinComponentType(const std::string& name);

}  // namespace srp::circuit
