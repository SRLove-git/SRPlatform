#include "bridge/car_entity.hpp"
#include "bridge/state_recorder.hpp"

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

TEST(BridgeStateRecorder, AppendsAndClearsSamples)
{
    srp::bridge::StateRecorder recorder;

    EXPECT_EQ(recorder.size(), 0U);
    EXPECT_FALSE(recorder.latest().has_value());

    srp::bridge::CarStateSample sample;
    sample.time_s = 0.1;
    sample.battery_voltage_v = 12.0;
    sample.motor_current_a = 0.5;
    sample.motor_angular_velocity_rad_s = 10.0;
    sample.chassis_position_x_m = 0.25;
    recorder.record(sample);

    EXPECT_EQ(recorder.size(), 1U);
    ASSERT_TRUE(recorder.latest().has_value());
    EXPECT_DOUBLE_EQ(recorder.latest()->time_s, 0.1);
    EXPECT_DOUBLE_EQ(recorder.latest()->battery_voltage_v, 12.0);
    EXPECT_DOUBLE_EQ(recorder.latest()->motor_current_a, 0.5);
    EXPECT_DOUBLE_EQ(recorder.latest()->motor_angular_velocity_rad_s, 10.0);
    EXPECT_DOUBLE_EQ(recorder.latest()->chassis_position_x_m, 0.25);

    recorder.clear();
    EXPECT_EQ(recorder.size(), 0U);
    EXPECT_FALSE(recorder.latest().has_value());
}

TEST(BridgeCarEntityStateRecorder, RecordsObservableStateAfterEachStep)
{
    srp::bridge::CarEntity car(testCarParameters());
    car.setThrottle(1.0);

    car.step(1.0 / 60.0);
    car.step(1.0 / 60.0);

    const srp::bridge::StateRecorder& recorder = car.recorder();
    EXPECT_EQ(recorder.size(), 2U);
    EXPECT_NEAR(car.elapsedTime(), 2.0 / 60.0, 1e-12);

    ASSERT_TRUE(recorder.latest().has_value());
    const srp::bridge::CarStateSample sample = *recorder.latest();
    EXPECT_NEAR(sample.time_s, 2.0 / 60.0, 1e-12);
    EXPECT_GT(sample.battery_voltage_v, 0.0);
    EXPECT_GT(sample.motor_current_a, 0.0);
    EXPECT_GT(sample.motor_angular_velocity_rad_s, 0.0);
    EXPECT_GT(sample.chassis_position_x_m, 0.0);
}
