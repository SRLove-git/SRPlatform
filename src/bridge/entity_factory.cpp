#include "bridge/entity_factory.hpp"

#include "bridge/car_entity.hpp"
#include "bridge/drone_entity.hpp"

#include <utility>

namespace srp::bridge
{
namespace
{

void applyNumber(
    const nlohmann::json& json,
    const char* key,
    double& value)
{
    const auto it = json.find(key);
    if (it != json.end() && it->is_number())
    {
        value = it->get<double>();
    }
}

CarParameters carParametersFromJson(const nlohmann::json& json)
{
    CarParameters parameters;

    applyNumber(json, "wheel_radius", parameters.wheel_radius);
    applyNumber(json, "chassis_mass", parameters.chassis_mass);
    applyNumber(json, "wheel_mass", parameters.wheel_mass);
    applyNumber(
        json,
        "rolling_resistance_torque_nm",
        parameters.rolling_resistance_torque_nm);
    applyNumber(
        json,
        "viscous_load_nm_per_rad_s",
        parameters.viscous_load_nm_per_rad_s);

    if (const auto battery_it = json.find("battery");
        battery_it != json.end() && battery_it->is_object())
    {
        const nlohmann::json& battery = *battery_it;
        applyNumber(
            battery,
            "full_charge_voltage_v",
            parameters.battery.full_charge_voltage_v);
        applyNumber(
            battery,
            "empty_charge_voltage_v",
            parameters.battery.empty_charge_voltage_v);
        applyNumber(
            battery,
            "internal_resistance_ohm",
            parameters.battery.internal_resistance_ohm);
        applyNumber(
            battery,
            "capacity_coulombs",
            parameters.battery.capacity_coulombs);
        applyNumber(
            battery,
            "initial_state_of_charge",
            parameters.battery.initial_state_of_charge);
    }

    if (const auto motor_it = json.find("motor");
        motor_it != json.end() && motor_it->is_object())
    {
        const nlohmann::json& motor = *motor_it;
        applyNumber(
            motor,
            "armature_resistance_ohm",
            parameters.motor.armature_resistance_ohm);
        applyNumber(
            motor,
            "armature_inductance_h",
            parameters.motor.armature_inductance_h);
        applyNumber(
            motor,
            "torque_constant_nm_per_a",
            parameters.motor.torque_constant_nm_per_a);
        applyNumber(
            motor,
            "back_emf_constant_v_per_rad_s",
            parameters.motor.back_emf_constant_v_per_rad_s);
        applyNumber(
            motor,
            "rotor_inertia_kg_m2",
            parameters.motor.rotor_inertia_kg_m2);
        applyNumber(
            motor,
            "viscous_friction_nm_per_rad_s",
            parameters.motor.viscous_friction_nm_per_rad_s);
    }

    return parameters;
}

DroneParameters droneParametersFromJson(const nlohmann::json& json)
{
    DroneParameters parameters;

    applyNumber(json, "chassis_mass", parameters.chassis_mass);
    applyNumber(
        json,
        "max_rotor_angular_velocity_rad_s",
        parameters.max_rotor_angular_velocity_rad_s);

    if (const auto quadcopter_it = json.find("quadcopter");
        quadcopter_it != json.end() && quadcopter_it->is_object())
    {
        const nlohmann::json& quadcopter = *quadcopter_it;
        applyNumber(
            quadcopter,
            "arm_length_m",
            parameters.quadcopter.arm_length_m);

        if (const auto propeller_it = quadcopter.find("propeller");
            propeller_it != quadcopter.end() && propeller_it->is_object())
        {
            const nlohmann::json& propeller = *propeller_it;
            applyNumber(
                propeller,
                "diameter_m",
                parameters.quadcopter.propeller.diameter_m);
            applyNumber(
                propeller,
                "thrust_coefficient",
                parameters.quadcopter.propeller.thrust_coefficient);
            applyNumber(
                propeller,
                "torque_coefficient",
                parameters.quadcopter.propeller.torque_coefficient);
            applyNumber(
                propeller,
                "air_density_kg_m3",
                parameters.quadcopter.propeller.air_density_kg_m3);
        }
    }

    return parameters;
}

}  // namespace

std::unique_ptr<IEntity> createEntity(
    const mod::EntityBlueprint& blueprint,
    std::string& error)
{
    error.clear();

    if (blueprint.kind == "car")
    {
        return std::make_unique<CarEntity>(
            carParametersFromJson(blueprint.parameters));
    }
    if (blueprint.kind == "drone")
    {
        return std::make_unique<DroneEntity>(
            droneParametersFromJson(blueprint.parameters));
    }

    error = "unsupported entity kind: " + blueprint.kind;
    return nullptr;
}

}  // namespace srp::bridge
