#include "bridge/battery_model.hpp"

#include <gtest/gtest.h>

namespace
{

srp::bridge::BatteryParameters testBatteryParameters()
{
    srp::bridge::BatteryParameters parameters;
    parameters.full_charge_voltage_v = 4.2;
    parameters.empty_charge_voltage_v = 3.0;
    parameters.internal_resistance_ohm = 0.05;
    parameters.capacity_coulombs = 3600.0;
    parameters.initial_state_of_charge = 1.0;
    return parameters;
}

}  // namespace

TEST(BridgeBatteryModel, OpenCircuitVoltageFollowsStateOfCharge)
{
    srp::bridge::BatteryParameters parameters = testBatteryParameters();
    parameters.initial_state_of_charge = 0.5;
    srp::bridge::BatteryModel battery(parameters);

    EXPECT_NEAR(battery.stateOfCharge(), 0.5, 1e-12);
    EXPECT_NEAR(battery.openCircuitVoltage(), 3.6, 1e-12);
}

TEST(BridgeBatteryModel, TerminalVoltageDropsWithLoad)
{
    srp::bridge::BatteryModel battery(testBatteryParameters());

    EXPECT_NEAR(battery.openCircuitVoltage(), 4.2, 1e-12);
    EXPECT_NEAR(battery.terminalVoltage(0.0), 4.2, 1e-12);
    EXPECT_NEAR(battery.terminalVoltage(2.0), 4.1, 1e-12);
    EXPECT_NEAR(battery.terminalVoltage(10.0), 3.7, 1e-12);
}

TEST(BridgeBatteryModel, DischargingReducesChargeAndVoltage)
{
    srp::bridge::BatteryModel battery(testBatteryParameters());

    battery.step(1.0, 1800.0);

    EXPECT_NEAR(battery.stateOfCharge(), 0.5, 1e-12);
    EXPECT_NEAR(battery.openCircuitVoltage(), 3.6, 1e-12);
    EXPECT_NEAR(battery.terminalVoltage(1.0), 3.55, 1e-12);
}

TEST(BridgeBatteryModel, ChargingRestoresChargeUpToCapacity)
{
    srp::bridge::BatteryParameters parameters = testBatteryParameters();
    parameters.initial_state_of_charge = 0.25;
    srp::bridge::BatteryModel battery(parameters);

    battery.step(-2.0, 2000.0);

    EXPECT_NEAR(battery.stateOfCharge(), 1.0, 1e-12);
    EXPECT_NEAR(battery.remainingChargeCoulombs(), battery.capacityCoulombs(), 1e-12);
}

TEST(BridgeBatteryModel, DischargeCannotGoBelowEmpty)
{
    srp::bridge::BatteryModel battery(testBatteryParameters());

    battery.step(10.0, 1000.0);

    EXPECT_NEAR(battery.stateOfCharge(), 0.0, 1e-12);
    EXPECT_NEAR(battery.openCircuitVoltage(), 3.0, 1e-12);
}

TEST(BridgeBatteryModel, NonPositiveTimeStepDoesNotChangeState)
{
    srp::bridge::BatteryModel battery(testBatteryParameters());

    battery.step(2.0, 0.0);
    battery.step(2.0, -0.1);

    EXPECT_NEAR(battery.stateOfCharge(), 1.0, 1e-12);
}

TEST(BridgeBatteryModel, ResetRestoresInitialState)
{
    srp::bridge::BatteryParameters parameters = testBatteryParameters();
    parameters.initial_state_of_charge = 0.8;
    srp::bridge::BatteryModel battery(parameters);

    battery.step(1.0, 500.0);
    battery.reset();

    EXPECT_NEAR(battery.stateOfCharge(), 0.8, 1e-12);
}

TEST(BridgeBatteryModel, RejectsInvalidParameters)
{
    srp::bridge::BatteryParameters parameters = testBatteryParameters();
    parameters.capacity_coulombs = 0.0;
    EXPECT_THROW(srp::bridge::BatteryModel{parameters}, std::invalid_argument);

    parameters = testBatteryParameters();
    parameters.internal_resistance_ohm = -0.1;
    EXPECT_THROW(srp::bridge::BatteryModel{parameters}, std::invalid_argument);

    parameters = testBatteryParameters();
    parameters.initial_state_of_charge = 1.1;
    EXPECT_THROW(srp::bridge::BatteryModel{parameters}, std::invalid_argument);
}
