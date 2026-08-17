#include "editor/circuit_editor.hpp"

#include "circuit/netlist_loader.hpp"

#include <gtest/gtest.h>

namespace srp::editor
{
namespace
{

using srp::circuit::Component;
using srp::circuit::ComponentId;
using srp::circuit::ComponentType;
using srp::circuit::Port;
using srp::circuit::PortId;

TEST(CircuitEditorTest, AddComponentStoresPositionAndDefaults)
{
    CircuitEditor editor;
    const ComponentId id = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(3.0, 4.0));

    ASSERT_NE(id, srp::circuit::kInvalidComponentId);
    EXPECT_EQ(editor.componentPosition(id), srp::math::Vec2(3.0, 4.0));

    const Component* component = editor.circuit().component(id);
    ASSERT_NE(component, nullptr);
    EXPECT_EQ(component->definition.type, ComponentType::kResistor);
    EXPECT_EQ(component->ports.size(), 2u);
    EXPECT_EQ(editor.portPosition(component->ports[0]),
              srp::math::Vec2(2.1, 4.0));
    EXPECT_EQ(editor.portPosition(component->ports[1]),
              srp::math::Vec2(3.9, 4.0));
}

TEST(CircuitEditorTest, WireCreatesSharedNode)
{
    CircuitEditor editor;
    const ComponentId left = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(0.0, 0.0));
    const ComponentId right = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(3.0, 0.0));

    const Component* left_component = editor.circuit().component(left);
    const Component* right_component = editor.circuit().component(right);
    const PortId left_port = left_component->ports[1];
    const PortId right_port = right_component->ports[0];

    EXPECT_TRUE(editor.wire(left_port, right_port));

    const Port* left_port_ptr = editor.circuit().port(left_port);
    const Port* right_port_ptr = editor.circuit().port(right_port);
    ASSERT_NE(left_port_ptr->node, srp::circuit::kInvalidNodeId);
    EXPECT_EQ(left_port_ptr->node, right_port_ptr->node);
    EXPECT_EQ(editor.circuit().node(left_port_ptr->node)->ports.size(), 2u);

    // Wiring the same pair again is a no-op success.
    EXPECT_TRUE(editor.wire(left_port, right_port));
}

TEST(CircuitEditorTest, WireReusesExistingNode)
{
    CircuitEditor editor;
    const ComponentId first = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(0.0, 0.0));
    const ComponentId second = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(3.0, 0.0));
    const ComponentId third = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(6.0, 0.0));

    const PortId first_out = editor.circuit().component(first)->ports[1];
    const PortId second_in = editor.circuit().component(second)->ports[0];
    const PortId third_in = editor.circuit().component(third)->ports[0];

    EXPECT_TRUE(editor.wire(first_out, second_in));
    EXPECT_TRUE(editor.wire(third_in, first_out));

    const Port* first_port = editor.circuit().port(first_out);
    const Port* third_port = editor.circuit().port(third_in);
    EXPECT_EQ(first_port->node, third_port->node);
    EXPECT_EQ(editor.circuit().node(first_port->node)->ports.size(), 3u);
}

TEST(CircuitEditorTest, RemoveComponentPrunesEmptyNodes)
{
    CircuitEditor editor;
    const ComponentId first = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(0.0, 0.0));
    const ComponentId second = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(3.0, 0.0));

    EXPECT_TRUE(editor.wire(
        editor.circuit().component(first)->ports[1],
        editor.circuit().component(second)->ports[0]));

    EXPECT_TRUE(editor.removeComponent(first));
    EXPECT_EQ(editor.circuit().component(first), nullptr);
    EXPECT_NE(editor.circuit().component(second), nullptr);

    // The shared node lost one port but still belongs to the second
    // component; it must survive. Removing the second one prunes it.
    EXPECT_EQ(editor.circuit().nodes().size(), 2u);
    EXPECT_TRUE(editor.removeComponent(second));
    EXPECT_EQ(editor.circuit().nodes().size(), 1u);
    EXPECT_EQ(editor.circuit().nodes().front().id, srp::circuit::kGroundNodeId);
}

TEST(CircuitEditorTest, UnwireDetachesAndPrunes)
{
    CircuitEditor editor;
    const ComponentId first = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(0.0, 0.0));
    const ComponentId second = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(3.0, 0.0));

    const PortId first_port = editor.circuit().component(first)->ports[1];
    const PortId second_port = editor.circuit().component(second)->ports[0];
    EXPECT_TRUE(editor.wire(first_port, second_port));

    EXPECT_TRUE(editor.unwire(first_port));
    EXPECT_EQ(editor.circuit().port(first_port)->node, srp::circuit::kInvalidNodeId);
    EXPECT_NE(editor.circuit().port(second_port)->node, srp::circuit::kInvalidNodeId);

    EXPECT_TRUE(editor.unwire(second_port));
    EXPECT_EQ(editor.circuit().nodes().size(), 1u);
}

TEST(CircuitEditorTest, ClearRemovesAllComponents)
{
    CircuitEditor editor;
    editor.addComponent(ComponentType::kResistor, srp::math::Vec2(0.0, 0.0));
    editor.addComponent(ComponentType::kVoltageSource, srp::math::Vec2(3.0, 0.0));

    editor.clear();

    EXPECT_TRUE(editor.circuit().components().empty());
    EXPECT_TRUE(editor.circuit().ports().empty());
    EXPECT_EQ(editor.circuit().nodes().size(), 1u);
    EXPECT_FALSE(editor.selected().has_value());
}

TEST(CircuitEditorTest, NetlistRoundTrip)
{
    CircuitEditor editor;
    const ComponentId source = editor.addComponent(
        ComponentType::kVoltageSource,
        srp::math::Vec2(0.0, 0.0));
    const ComponentId resistor = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(3.0, 0.0));

    const Component* source_component = editor.circuit().component(source);
    const Component* resistor_component = editor.circuit().component(resistor);
    EXPECT_TRUE(editor.wire(source_component->ports[0], resistor_component->ports[0]));
    EXPECT_TRUE(editor.wire(source_component->ports[1], resistor_component->ports[1]));

    const nlohmann::json json = editor.toNetlistJson();
    EXPECT_EQ(json["components"].size(), 2u);

    std::string error;
    const auto loaded = srp::circuit::loadNetlist(json, error);
    ASSERT_TRUE(loaded.has_value()) << error;
    EXPECT_EQ(loaded->components().size(), 2u);

    const Component& loaded_source = loaded->components().front();
    const Component& loaded_resistor = loaded->components().back();
    EXPECT_EQ(loaded_source.definition.type, ComponentType::kVoltageSource);
    EXPECT_EQ(loaded_resistor.definition.type, ComponentType::kResistor);
    EXPECT_EQ(
        loaded->port(loaded_source.ports[0])->node,
        loaded->port(loaded_resistor.ports[0])->node);
}

TEST(CircuitEditorTest, LoadNetlistJsonRebuildsEditor)
{
    CircuitEditor editor;
    const ComponentId source = editor.addComponent(
        ComponentType::kVoltageSource,
        srp::math::Vec2(0.0, 0.0));
    const ComponentId resistor = editor.addComponent(
        ComponentType::kResistor,
        srp::math::Vec2(3.0, 0.0));
    EXPECT_TRUE(editor.wire(
        editor.circuit().component(source)->ports[0],
        editor.circuit().component(resistor)->ports[0]));

    const nlohmann::json json = editor.toNetlistJson();

    CircuitEditor rebuilt;
    std::string error;
    EXPECT_TRUE(rebuilt.loadNetlistJson(json, error)) << error;
    EXPECT_EQ(rebuilt.circuit().components().size(), 2u);
    EXPECT_EQ(rebuilt.circuit().ports().size(), 4u);
    EXPECT_EQ(
        rebuilt.componentPosition(rebuilt.circuit().components()[1].id),
        srp::math::Vec2(3.0, 0.0));
}

}  // namespace
}  // namespace srp::editor
