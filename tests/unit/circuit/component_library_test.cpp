#include "circuit/component_library.hpp"
#include "circuit/netlist_loader.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <variant>

namespace
{

srp::circuit::ComponentDefinition inductorDefinition(double inductance)
{
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kInductor;
    definition.name = "custom_inductor";
    definition.port_names =
        srp::circuit::defaultPortNames(srp::circuit::ComponentType::kInductor);
    definition.parameters = srp::circuit::InductorParameters{inductance};
    return definition;
}

}  // namespace

TEST(ComponentLibrary, RegistersAndInstantiatesCustomComponent)
{
    srp::circuit::ComponentLibrary library;

    EXPECT_TRUE(library.registerComponent("motor_winding", inductorDefinition(0.02)));
    EXPECT_TRUE(library.contains("motor_winding"));
    EXPECT_EQ(library.size(), 1U);

    const auto definition = library.find("motor_winding");
    ASSERT_TRUE(definition.has_value());
    EXPECT_EQ(definition->type, srp::circuit::ComponentType::kInductor);
    EXPECT_EQ(definition->port_names.size(), 2U);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::InductorParameters>(
            definition->parameters).inductance,
        0.02);

    srp::circuit::CircuitModel circuit;
    const auto id = library.instantiate(circuit, "motor_winding");
    ASSERT_TRUE(id.has_value());
    const srp::circuit::Component* component = circuit.component(*id);
    ASSERT_NE(component, nullptr);
    EXPECT_EQ(component->definition.type, srp::circuit::ComponentType::kInductor);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::InductorParameters>(
            component->definition.parameters).inductance,
        0.02);
}

TEST(ComponentLibrary, RejectsDuplicateAndBuiltinNames)
{
    srp::circuit::ComponentLibrary library;

    EXPECT_TRUE(library.registerComponent("my_part", inductorDefinition(0.01)));
    EXPECT_FALSE(library.registerComponent("my_part", inductorDefinition(0.02)));
    EXPECT_FALSE(library.registerComponent("resistor", inductorDefinition(0.03)));
}

TEST(ComponentLibrary, UnregistersCustomComponents)
{
    srp::circuit::ComponentLibrary library;

    EXPECT_TRUE(library.registerComponent("temp_part", inductorDefinition(0.05)));
    EXPECT_TRUE(library.contains("temp_part"));
    EXPECT_TRUE(library.unregisterComponent("temp_part"));
    EXPECT_FALSE(library.contains("temp_part"));
    EXPECT_FALSE(library.unregisterComponent("temp_part"));
}

TEST(ComponentLibrary, BuiltinLookupWorks)
{
    srp::circuit::ComponentLibrary library;

    const auto resistor = library.find("resistor");
    ASSERT_TRUE(resistor.has_value());
    EXPECT_EQ(resistor->type, srp::circuit::ComponentType::kResistor);
    EXPECT_EQ(resistor->port_names.size(), 2U);

    const auto pwm = library.find("pwm_source");
    ASSERT_TRUE(pwm.has_value());
    EXPECT_EQ(pwm->type, srp::circuit::ComponentType::kPwmSource);
    EXPECT_EQ(pwm->port_names.size(), 1U);

    EXPECT_FALSE(library.find("quantum_resistor").has_value());
}

TEST(ComponentLibrary, NamesListsOnlyCustomEntries)
{
    srp::circuit::ComponentLibrary library;
    library.registerComponent("alpha", inductorDefinition(0.1));
    library.registerComponent("beta", inductorDefinition(0.2));

    const std::vector<std::string> names = library.names();
    EXPECT_EQ(names.size(), 2U);
    EXPECT_NE(
        std::find(names.begin(), names.end(), "alpha"),
        names.end());
    EXPECT_NE(
        std::find(names.begin(), names.end(), "beta"),
        names.end());
}

TEST(ComponentLibrary, RejectsMismatchedRegistration)
{
    srp::circuit::ComponentLibrary library;
    srp::circuit::ComponentDefinition definition;
    definition.type = srp::circuit::ComponentType::kResistor;
    definition.parameters = srp::circuit::CapacitorParameters{};

    EXPECT_FALSE(library.registerComponent("broken", definition));
}

TEST(ComponentLibrary, NetlistUsesCustomComponentTemplate)
{
    srp::circuit::ComponentLibrary library;
    srp::circuit::ComponentDefinition white_led;
    white_led.type = srp::circuit::ComponentType::kDiode;
    white_led.name = "white_led";
    white_led.port_names =
        srp::circuit::defaultPortNames(srp::circuit::ComponentType::kDiode);
    srp::circuit::DiodeParameters diode;
    diode.forward_voltage = 2.8;
    white_led.parameters = diode;
    EXPECT_TRUE(library.registerComponent("white_led", white_led));

    nlohmann::json netlist;
    netlist["components"] = nlohmann::json::array();
    nlohmann::json source;
    source["type"] = "voltage_source";
    source["name"] = "V1";
    source["ports"] = nlohmann::json::array({"vcc", "gnd"});
    source["parameters"]["voltage"] = 12.0;
    netlist["components"].push_back(source);

    nlohmann::json led;
    led["type"] = "white_led";
    led["name"] = "LED1";
    led["ports"] = nlohmann::json::array({"vcc", "gnd"});
    netlist["components"].push_back(led);

    std::string error;
    const auto circuit = srp::circuit::loadNetlist(netlist, library, error);

    ASSERT_TRUE(circuit.has_value()) << error;
    ASSERT_EQ(circuit->components().size(), 2U);
    const srp::circuit::Component* led_component = circuit->component(2);
    ASSERT_NE(led_component, nullptr);
    EXPECT_EQ(led_component->definition.type, srp::circuit::ComponentType::kDiode);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::DiodeParameters>(
            led_component->definition.parameters).forward_voltage,
        2.8);
}

TEST(ComponentLibrary, NetlistParametersOverrideTemplate)
{
    srp::circuit::ComponentLibrary library;
    srp::circuit::ComponentDefinition motor;
    motor.type = srp::circuit::ComponentType::kInductor;
    motor.port_names =
        srp::circuit::defaultPortNames(srp::circuit::ComponentType::kInductor);
    motor.parameters = srp::circuit::InductorParameters{0.02};
    EXPECT_TRUE(library.registerComponent("motor_winding", motor));

    nlohmann::json netlist;
    nlohmann::json entry;
    entry["type"] = "motor_winding";
    entry["name"] = "M1";
    entry["ports"] = nlohmann::json::array({"a", "b"});
    entry["parameters"]["inductance"] = 0.05;
    netlist["components"] = nlohmann::json::array({entry});

    std::string error;
    const auto circuit = srp::circuit::loadNetlist(netlist, library, error);

    ASSERT_TRUE(circuit.has_value()) << error;
    ASSERT_EQ(circuit->components().size(), 1U);
    const srp::circuit::Component* component = circuit->component(1);
    ASSERT_NE(component, nullptr);
    EXPECT_DOUBLE_EQ(
        std::get<srp::circuit::InductorParameters>(
            component->definition.parameters).inductance,
        0.05);
}

TEST(ComponentLibrary, NetlistRejectsUnknownTypeWithLibrary)
{
    srp::circuit::ComponentLibrary library;
    nlohmann::json netlist;
    nlohmann::json entry;
    entry["type"] = "not_registered_anywhere";
    entry["ports"] = nlohmann::json::array({"a", "b"});
    netlist["components"] = nlohmann::json::array({entry});

    std::string error;
    EXPECT_FALSE(srp::circuit::loadNetlist(netlist, library, error).has_value());
    EXPECT_FALSE(error.empty());
}
