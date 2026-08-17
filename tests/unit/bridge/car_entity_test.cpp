#include "bridge/car_entity.hpp"
#include "physics/rigid_body.hpp"

#include <gtest/gtest.h>

namespace
{

srp::bridge::CarParameters testCarParameters()
{
    srp::bridge::CarParameters parameters;
    parameters.battery.full_charge_voltage_v = 12.0;
    parameters.battery.empty_charge_voltage_v = 9.0;
    parameters.battery.internal_resistance_ohm = 0.05;
    parameters.battery.capacity_coulombs = 3600.0;
    parameters.battery.initial_state_of_charge = 1.0;

    parameters.motor.armature_resistance_ohm = 0.5;
    parameters.motor.armature_inductance_h = 0.001;
    parameters.motor.torque_constant_nm_per_a = 0.05;
    parameters.motor.back_emf_constant_v_per_rad_s = 0.05;
    parameters.motor.rotor_inertia_kg_m2 = 0.0001;
    parameters.motor.viscous_friction_nm_per_rad_s = 0.0;

    parameters.wheel_radius = 0.3;
    parameters.chassis_mass = 1.0;
    parameters.wheel_mass = 1.0;
    parameters.rolling_resistance_torque_nm = 0.01;
    parameters.viscous_load_nm_per_rad_s = 0.0005;
    return parameters;
}

}  // namespace

TEST(BridgeCarEntity, PositiveThrottleMovesVehicleForward)
{
    srp::bridge::CarEntity car(testCarParameters());
    car.setThrottle(1.0);

    car.step(1.0 / 60.0);
    car.step(1.0 / 60.0);

    const auto* chassis = car.chassisBody();
    const auto* wheel = car.wheelBody();
    ASSERT_NE(chassis, nullptr);
    ASSERT_NE(wheel, nullptr);

    EXPECT_GT(chassis->position.x, 0.0);
    EXPECT_GT(wheel->angular_velocity.z, 0.0);
    EXPECT_GT(car.motorCurrent(), 0.0);
    EXPECT_GT(car.motorAngularVelocity(), 0.0);
    EXPECT_LT(car.batteryStateOfCharge(), 1.0);
}

TEST(BridgeCarEntity, NegativeThrottleMovesVehicleBackward)
{
    srp::bridge::CarEntity car(testCarParameters());
    car.setThrottle(-1.0);

    car.step(1.0 / 60.0);
    car.step(1.0 / 60.0);

    const auto* chassis = car.chassisBody();
    const auto* wheel = car.wheelBody();
    ASSERT_NE(chassis, nullptr);
    ASSERT_NE(wheel, nullptr);

    EXPECT_LT(chassis->position.x, 0.0);
    EXPECT_LT(wheel->angular_velocity.z, 0.0);
    EXPECT_LT(car.batteryStateOfCharge(), 1.0);
}

TEST(BridgeCarEntity, ZeroThrottleDoesNotDriveVehicle)
{
    srp::bridge::CarEntity car(testCarParameters());
    car.setThrottle(0.0);

    car.step(1.0 / 60.0);

    const auto* chassis = car.chassisBody();
    ASSERT_NE(chassis, nullptr);
    EXPECT_NEAR(chassis->position.x, 0.0, 1e-12);
}

TEST(BridgeCarEntity, RejectsInvalidVehicleParameters)
{
    srp::bridge::CarParameters parameters = testCarParameters();
    parameters.wheel_radius = 0.0;
    EXPECT_THROW(srp::bridge::CarEntity{parameters}, std::invalid_argument);

    parameters = testCarParameters();
    parameters.chassis_mass = 0.0;
    EXPECT_THROW(srp::bridge::CarEntity{parameters}, std::invalid_argument);
}
