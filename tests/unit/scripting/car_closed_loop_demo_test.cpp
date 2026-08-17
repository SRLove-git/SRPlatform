#include "scripting/car_closed_loop_demo.hpp"
#include "scripting/lua_script_host.hpp"

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

TEST(CarClosedLoopDemo, ScriptDrivesForwardThenStopsAndSteers)
{
    srp::scripting::CarClosedLoopDemo demo(testCarParameters());

    constexpr const char* script =
        "elapsed = 0\n"
        "function update(dt)\n"
        "    elapsed = elapsed + dt\n"
        "    if elapsed < 1.0 then\n"
        "        set_motor(1, 1.0)\n"
        "    else\n"
        "        set_motor(1, 0.0)\n"
        "    end\n"
        "    set_servo(1, 0.5)\n"
        "end\n";

    EXPECT_TRUE(demo.loadScript("controller", script));

    for (int step = 0; step < 120; ++step)
    {
        EXPECT_TRUE(demo.step(1.0 / 60.0));
    }

    const auto* chassis = demo.car().chassisBody();
    ASSERT_NE(chassis, nullptr);

    EXPECT_GT(chassis->position.x, 0.0);
    EXPECT_DOUBLE_EQ(demo.car().throttle(), 0.0);
    EXPECT_DOUBLE_EQ(demo.car().steering(), 0.5);
    EXPECT_GT(demo.car().heading(), 0.0);
    EXPECT_EQ(demo.car().recorder().size(), 120U);
}

TEST(CarClosedLoopDemo, ScriptCanReadCarSensors)
{
    srp::scripting::CarClosedLoopDemo demo(testCarParameters());

    constexpr const char* script =
        "observed_speed = 0\n"
        "observed_position = 0\n"
        "observed_heading = 0\n"
        "function update(dt)\n"
        "    set_motor(1, 1.0)\n"
        "    observed_speed = read_sensor(1)\n"
        "    observed_position = read_sensor(2)\n"
        "    observed_heading = read_sensor(3)\n"
        "end\n";

    EXPECT_TRUE(demo.loadScript("sensors", script));
    EXPECT_TRUE(demo.step(1.0 / 60.0));

    const auto speed = demo.host().getNumber("sensors", "observed_speed");
    const auto position = demo.host().getNumber("sensors", "observed_position");
    const auto heading = demo.host().getNumber("sensors", "observed_heading");

    ASSERT_TRUE(speed.has_value());
    ASSERT_TRUE(position.has_value());
    ASSERT_TRUE(heading.has_value());
    EXPECT_GE(*speed, 0.0);
    EXPECT_GE(*position, 0.0);
    EXPECT_GE(*heading, 0.0);
}
