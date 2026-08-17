#include "bridge/dc_motor_model.hpp"

#include <stdexcept>

namespace srp::bridge
{

DcMotorModel::DcMotorModel(const DcMotorParameters& parameters)
    : parameters_(parameters)
{
    if (parameters.armature_resistance_ohm <= 0.0)
    {
        throw std::invalid_argument("motor armature resistance must be positive");
    }

    if (parameters.armature_inductance_h <= 0.0)
    {
        throw std::invalid_argument("motor armature inductance must be positive");
    }

    if (parameters.torque_constant_nm_per_a <= 0.0)
    {
        throw std::invalid_argument("motor torque constant must be positive");
    }

    if (parameters.back_emf_constant_v_per_rad_s <= 0.0)
    {
        throw std::invalid_argument("motor back-EMF constant must be positive");
    }

    if (parameters.rotor_inertia_kg_m2 <= 0.0)
    {
        throw std::invalid_argument("motor rotor inertia must be positive");
    }

    if (parameters.viscous_friction_nm_per_rad_s < 0.0)
    {
        throw std::invalid_argument("motor viscous friction cannot be negative");
    }
}

double DcMotorModel::current() const
{
    return current_a_;
}

double DcMotorModel::angularVelocity() const
{
    return angular_velocity_rad_s_;
}

double DcMotorModel::shaftAngle() const
{
    return shaft_angle_rad_;
}

double DcMotorModel::backEmf() const
{
    return parameters_.back_emf_constant_v_per_rad_s * angular_velocity_rad_s_;
}

double DcMotorModel::electricalTorque() const
{
    return parameters_.torque_constant_nm_per_a * current_a_;
}

void DcMotorModel::step(double supply_voltage_v, double load_torque_nm, double dt_s)
{
    if (dt_s <= 0.0)
    {
        return;
    }

    const double resistance = parameters_.armature_resistance_ohm;
    const double inductance = parameters_.armature_inductance_h;
    const double torque_constant = parameters_.torque_constant_nm_per_a;
    const double back_emf_constant = parameters_.back_emf_constant_v_per_rad_s;
    const double inertia = parameters_.rotor_inertia_kg_m2;
    const double viscous_friction = parameters_.viscous_friction_nm_per_rad_s;

    // Backward Euler discretization of the coupled electrical and mechanical
    // equations. The resulting 2x2 system has a strictly positive determinant
    // for valid parameters.
    const double a = inductance / dt_s + resistance;
    const double c = inertia / dt_s + viscous_friction;

    const double rhs_current = inductance / dt_s * current_a_ + supply_voltage_v;
    const double rhs_omega = inertia / dt_s * angular_velocity_rad_s_ - load_torque_nm;

    const double determinant = a * c + torque_constant * back_emf_constant;
    const double next_current =
        (rhs_current * c - back_emf_constant * rhs_omega) / determinant;
    const double next_angular_velocity =
        (torque_constant * rhs_current + a * rhs_omega) / determinant;

    shaft_angle_rad_ += 0.5 * (angular_velocity_rad_s_ + next_angular_velocity) * dt_s;
    current_a_ = next_current;
    angular_velocity_rad_s_ = next_angular_velocity;
}

void DcMotorModel::reset()
{
    current_a_ = 0.0;
    angular_velocity_rad_s_ = 0.0;
    shaft_angle_rad_ = 0.0;
}

}  // namespace srp::bridge
