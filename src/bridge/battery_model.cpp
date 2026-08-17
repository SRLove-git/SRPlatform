#include "bridge/battery_model.hpp"

#include <algorithm>
#include <stdexcept>

namespace srp::bridge
{

namespace
{

double validateStateOfCharge(double value)
{
    if (value < 0.0 || value > 1.0)
    {
        throw std::invalid_argument("battery state of charge must be within [0, 1]");
    }

    return value;
}

}  // namespace

BatteryModel::BatteryModel(const BatteryParameters& parameters)
    : parameters_(parameters),
      charge_coulombs_(validateStateOfCharge(parameters.initial_state_of_charge) *
                       parameters.capacity_coulombs)
{
    if (parameters.full_charge_voltage_v <= parameters.empty_charge_voltage_v)
    {
        throw std::invalid_argument(
            "full-charge voltage must be greater than empty-charge voltage");
    }

    if (parameters.internal_resistance_ohm < 0.0)
    {
        throw std::invalid_argument("battery internal resistance cannot be negative");
    }

    if (parameters.capacity_coulombs <= 0.0)
    {
        throw std::invalid_argument("battery capacity must be positive");
    }
}

double BatteryModel::openCircuitVoltage() const
{
    const double fraction = stateOfCharge();
    return parameters_.empty_charge_voltage_v +
           (parameters_.full_charge_voltage_v - parameters_.empty_charge_voltage_v) *
               fraction;
}

double BatteryModel::terminalVoltage(double load_current_a) const
{
    const double voltage = openCircuitVoltage() -
                           load_current_a * parameters_.internal_resistance_ohm;
    return std::max(0.0, voltage);
}

double BatteryModel::stateOfCharge() const
{
    return charge_coulombs_ / parameters_.capacity_coulombs;
}

double BatteryModel::remainingChargeCoulombs() const
{
    return charge_coulombs_;
}

double BatteryModel::capacityCoulombs() const
{
    return parameters_.capacity_coulombs;
}

void BatteryModel::step(double load_current_a, double dt_s)
{
    if (dt_s <= 0.0)
    {
        return;
    }

    const double charge_delta = load_current_a * dt_s;
    charge_coulombs_ = std::clamp(
        charge_coulombs_ - charge_delta,
        0.0,
        parameters_.capacity_coulombs);
}

void BatteryModel::reset()
{
    charge_coulombs_ = parameters_.initial_state_of_charge *
                       parameters_.capacity_coulombs;
}

}  // namespace srp::bridge
