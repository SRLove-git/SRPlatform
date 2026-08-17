#pragma once

namespace srp::bridge
{

// Permanent-magnet brushed DC motor parameters. All values use SI units:
// ohms, henries, N*m/A, V/(rad/s), kg*m^2, and N*m/(rad/s).
struct DcMotorParameters
{
    double armature_resistance_ohm{0.5};
    double armature_inductance_h{0.001};
    double torque_constant_nm_per_a{0.05};
    double back_emf_constant_v_per_rad_s{0.05};
    double rotor_inertia_kg_m2{0.0001};
    double viscous_friction_nm_per_rad_s{0.0};
};

// Simplified DC motor model for the electromechanical bridge.
//
// The electrical equation is:
//     V = I*R + L*dI/dt + Ke*omega
// The mechanical equation is:
//     J*domega/dt = Kt*I - B*omega - load_torque
//
// Positive supply voltage drives the motor forward. Positive load torque
// opposes rotation.
class DcMotorModel
{
public:
    explicit DcMotorModel(const DcMotorParameters& parameters = {});

    double current() const;
    double angularVelocity() const;
    double shaftAngle() const;

    double backEmf() const;
    double electricalTorque() const;

    void step(double supply_voltage_v, double load_torque_nm, double dt_s);
    void reset();

private:
    DcMotorParameters parameters_;
    double current_a_{0.0};
    double angular_velocity_rad_s_{0.0};
    double shaft_angle_rad_{0.0};
};

}  // namespace srp::bridge
