#include "bridge/dc_motor_model.hpp"

#include <gtest/gtest.h>

namespace
{

constexpr double kTimeStep = 1e-4;

srp::bridge::DcMotorParameters testMotorParameters()
{
    srp::bridge::DcMotorParameters parameters;
    parameters.armature_resistance_ohm = 0.5;
    parameters.armature_inductance_h = 0.001;
    parameters.torque_constant_nm_per_a = 0.05;
    parameters.back_emf_constant_v_per_rad_s = 0.05;
    parameters.rotor_inertia_kg_m2 = 0.0001;
    parameters.viscous_friction_nm_per_rad_s = 0.0005;
    return parameters;
}

void runMotor(
    srp::bridge::DcMotorModel& motor,
    double supply_voltage_v,
    double load_torque_nm,
    double duration_s)
{
    const int step_count = static_cast<int>(duration_s / kTimeStep);
    for (int step = 0; step < step_count; ++step)
    {
        motor.step(supply_voltage_v, load_torque_nm, kTimeStep);
    }
}

}  // namespace

TEST(BridgeDcMotorModel, BackEmfAndTorqueFollowState)
{
    srp::bridge::DcMotorModel motor(testMotorParameters());

    runMotor(motor, 12.0, 0.0, 0.01);

    EXPECT_NEAR(
        motor.backEmf(),
        testMotorParameters().back_emf_constant_v_per_rad_s * motor.angularVelocity(),
        1e-12);
    EXPECT_NEAR(
        motor.electricalTorque(),
        testMotorParameters().torque_constant_nm_per_a * motor.current(),
        1e-12);
}

TEST(BridgeDcMotorModel, ReachesExpectedNoLoadSteadyState)
{
    srp::bridge::DcMotorModel motor(testMotorParameters());

    runMotor(motor, 12.0, 0.0, 2.0);

    const double expected_angular_velocity = 12.0 / 0.055;
    const double expected_current = 0.01 * expected_angular_velocity;

    EXPECT_NEAR(motor.angularVelocity(), expected_angular_velocity, 1e-6);
    EXPECT_NEAR(motor.current(), expected_current, 1e-6);
    EXPECT_NEAR(
        motor.current() * 0.5 + motor.backEmf(),
        12.0,
        1e-6);
    EXPECT_NEAR(
        motor.electricalTorque(),
        0.0005 * motor.angularVelocity(),
        1e-6);
}

TEST(BridgeDcMotorModel, LoadTorqueReducesSpeedAndIncreasesCurrent)
{
    srp::bridge::DcMotorModel no_load(testMotorParameters());
    srp::bridge::DcMotorModel loaded(testMotorParameters());

    runMotor(no_load, 12.0, 0.0, 2.0);
    runMotor(loaded, 12.0, 0.05, 2.0);

    const double expected_loaded_angular_velocity = 11.5 / 0.055;
    const double expected_loaded_current = 0.01 * expected_loaded_angular_velocity + 1.0;

    EXPECT_NEAR(loaded.angularVelocity(), expected_loaded_angular_velocity, 1e-6);
    EXPECT_NEAR(loaded.current(), expected_loaded_current, 1e-6);
    EXPECT_LT(loaded.angularVelocity(), no_load.angularVelocity());
    EXPECT_GT(loaded.current(), no_load.current());
}

TEST(BridgeDcMotorModel, ShaftAngleIntegratesRotation)
{
    srp::bridge::DcMotorModel motor(testMotorParameters());

    runMotor(motor, 12.0, 0.0, 0.01);

    EXPECT_GT(motor.shaftAngle(), 0.0);
}

TEST(BridgeDcMotorModel, NonPositiveTimeStepDoesNotChangeState)
{
    srp::bridge::DcMotorModel motor(testMotorParameters());

    motor.step(12.0, 0.0, 0.0);
    motor.step(12.0, 0.0, -0.1);

    EXPECT_DOUBLE_EQ(motor.current(), 0.0);
    EXPECT_DOUBLE_EQ(motor.angularVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(motor.shaftAngle(), 0.0);
}

TEST(BridgeDcMotorModel, ResetRestoresInitialState)
{
    srp::bridge::DcMotorModel motor(testMotorParameters());

    runMotor(motor, 12.0, 0.0, 0.5);
    motor.reset();

    EXPECT_DOUBLE_EQ(motor.current(), 0.0);
    EXPECT_DOUBLE_EQ(motor.angularVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(motor.shaftAngle(), 0.0);
}

TEST(BridgeDcMotorModel, RejectsInvalidParameters)
{
    srp::bridge::DcMotorParameters parameters = testMotorParameters();
    parameters.armature_resistance_ohm = 0.0;
    EXPECT_THROW(srp::bridge::DcMotorModel{parameters}, std::invalid_argument);

    parameters = testMotorParameters();
    parameters.armature_inductance_h = 0.0;
    EXPECT_THROW(srp::bridge::DcMotorModel{parameters}, std::invalid_argument);

    parameters = testMotorParameters();
    parameters.rotor_inertia_kg_m2 = 0.0;
    EXPECT_THROW(srp::bridge::DcMotorModel{parameters}, std::invalid_argument);

    parameters = testMotorParameters();
    parameters.viscous_friction_nm_per_rad_s = -0.1;
    EXPECT_THROW(srp::bridge::DcMotorModel{parameters}, std::invalid_argument);
}
