#pragma once

#include "circuit/circuit_types.hpp"

namespace srp::circuit
{

inline bool diodeConducts(Voltage branch_voltage, const DiodeParameters& parameters)
{
    return branch_voltage > parameters.forward_voltage;
}

inline double diodeConductance(
    const DiodeParameters& parameters,
    bool conducting)
{
    return 1.0 / (conducting ? parameters.on_resistance : parameters.off_resistance);
}

inline double diodeCurrentSource(
    const DiodeParameters& parameters,
    bool conducting)
{
    return conducting ? -diodeConductance(parameters, true) * parameters.forward_voltage : 0.0;
}

inline double switchConductance(const SwitchParameters& parameters)
{
    return 1.0 / (parameters.closed ? parameters.on_resistance : parameters.off_resistance);
}

inline double diodeCurrent(
    const DiodeParameters& parameters,
    bool conducting,
    Voltage branch_voltage)
{
    const double conductance = diodeConductance(parameters, conducting);
    return conducting
               ? conductance * (branch_voltage - parameters.forward_voltage)
               : conductance * branch_voltage;
}

inline double switchCurrent(
    const SwitchParameters& parameters,
    Voltage branch_voltage)
{
    return switchConductance(parameters) * branch_voltage;
}

}  // namespace srp::circuit
