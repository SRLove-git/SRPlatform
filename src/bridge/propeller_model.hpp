#pragma once

namespace srp::bridge
{

// Simplified propeller parameters. All values use SI units: meters,
// kg/m^3, and dimensionless thrust/torque coefficients.
struct PropellerParameters
{
    // Rotor diameter; the default matches a common 5-inch quadcopter prop.
    double diameter_m{0.127};

    // Dimensionless thrust coefficient C_T. Typical small props: 0.08-0.13.
    double thrust_coefficient{0.11};

    // Dimensionless torque coefficient C_Q. Typical small props: 0.008-0.012.
    double torque_coefficient{0.011};

    // Ambient air density; the default is sea-level standard atmosphere.
    double air_density_kg_m3{1.225};
};

// Simplified propeller model based on the standard momentum-theory rotor
// equations. With n = omega / (2*pi) revolutions per second:
//
//     T = C_T * rho * n * |n| * D^4
//     Q = C_Q * rho * n * |n| * D^5   (magnitude)
//
// The sign of the angular velocity is preserved so that a reversed spin
// produces reversed thrust along the rotor axis. Aerodynamic torque always
// opposes the direction of rotation.
class PropellerModel
{
public:
    explicit PropellerModel(const PropellerParameters& parameters = {});

    double angularVelocity() const;

    // Thrust along the rotor axis in newtons. Positive angular velocity
    // produces positive (upward) thrust.
    double thrust() const;

    // Aerodynamic drag torque opposing the spin direction in newton-meters.
    double torque() const;

    // Mechanical power required to keep the propeller spinning, in watts.
    double power() const;

    void setAngularVelocity(double angular_velocity);
    void reset();

private:
    PropellerParameters parameters_;
    double angular_velocity_rad_s_{0.0};
};

}  // namespace srp::bridge
