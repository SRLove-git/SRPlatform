#pragma once

namespace srp::bridge
{

// Battery parameters use SI units. The open-circuit voltage is modelled as a
// straight line between the empty and full state-of-charge voltages.
struct BatteryParameters
{
    double full_charge_voltage_v{4.2};
    double empty_charge_voltage_v{3.0};
    double internal_resistance_ohm{0.05};
    double capacity_coulombs{3600.0};
    double initial_state_of_charge{1.0};
};

// Simplified battery model for the electromechanical bridge.
//
// Positive load current discharges the battery. The terminal voltage is the
// open-circuit voltage minus the internal-resistance drop, clamped to zero.
class BatteryModel
{
public:
    explicit BatteryModel(const BatteryParameters& parameters = {});

    double openCircuitVoltage() const;
    double terminalVoltage(double load_current_a) const;

    double stateOfCharge() const;
    double remainingChargeCoulombs() const;
    double capacityCoulombs() const;

    void step(double load_current_a, double dt_s);
    void reset();

private:
    BatteryParameters parameters_;
    double charge_coulombs_;
};

}  // namespace srp::bridge
