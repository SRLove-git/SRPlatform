#pragma once

#include "circuit/circuit_model.hpp"
#include "core/math/types.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

namespace srp::editor
{

// Interactive circuit editor state on top of a CircuitModel. Components live
// on a 2D canvas; ports get derived positions and can be wired by connecting
// two ports (they share a node). Canvas coordinates are in logical units; the
// UI layer maps them to pixels.
class CircuitEditor
{
public:
    CircuitEditor();

    // Creates a component of the given type with default parameters at the
    // given canvas position. Returns kInvalidComponentId on failure.
    srp::circuit::ComponentId addComponent(
        srp::circuit::ComponentType type,
        const srp::math::Vec2& position);

    bool removeComponent(srp::circuit::ComponentId id);
    void clear();

    // Wires two ports so they share a node. When neither side has a node yet,
    // a fresh node is created. Returns false for unknown ports.
    bool wire(
        srp::circuit::PortId port_a,
        srp::circuit::PortId port_b);

    // Detaches a port from its node and prunes empty nodes.
    bool unwire(srp::circuit::PortId port_id);

    srp::circuit::CircuitModel& circuit();
    const srp::circuit::CircuitModel& circuit() const;

    srp::math::Vec2 componentPosition(srp::circuit::ComponentId id) const;
    bool setComponentPosition(
        srp::circuit::ComponentId id,
        const srp::math::Vec2& position);

    srp::math::Vec2 portPosition(srp::circuit::PortId id) const;

    bool select(srp::circuit::ComponentId id);
    void deselect();
    std::optional<srp::circuit::ComponentId> selected() const;

    void setPendingWirePort(srp::circuit::PortId id);
    std::optional<srp::circuit::PortId> pendingWirePort() const;
    void cancelWire();

    // Exports the current topology in the documented netlist JSON format.
    nlohmann::json toNetlistJson() const;

    // Replaces the current circuit with the topology from a netlist JSON
    // document and lays the components out on a grid. Returns false and sets
    // error when the document is invalid.
    bool loadNetlistJson(
        const nlohmann::json& json,
        std::string& error);

    // Port placement offsets relative to the component center.
    static srp::math::Vec2 portOffset(
        const srp::circuit::ComponentDefinition& definition,
        std::size_t port_index);

private:
    srp::circuit::CircuitModel circuit_;
    std::unordered_map<srp::circuit::ComponentId, srp::math::Vec2> positions_;
    std::optional<srp::circuit::ComponentId> selected_;
    std::optional<srp::circuit::PortId> pending_wire_port_;
};

}  // namespace srp::editor
