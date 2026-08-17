#pragma once

#include "bridge/propeller_model.hpp"
#include "core/math/types.hpp"

#include <array>
#include <cstddef>

namespace srp::bridge
{

// Quadcopter force-assembly parameters. All values use SI units.
struct QuadcopterParameters
{
    // Distance from the center of mass to each rotor axis, in meters.
    double arm_length_m{0.25};

    // All four rotors share the same propeller geometry.
    PropellerParameters propeller{};
};

// Assembles the net body-frame forces and torques produced by four rotors
// mounted in a "+" configuration around the center of mass:
//
//     index 0: front  at ( 0, 0, +L), spins +Y
//     index 1: right  at (+L, 0,  0), spins -Y
//     index 2: back   at ( 0, 0, -L), spins +Y
//     index 3: left   at (-L, 0,  0), spins -Y
//
// The body frame uses the project convention Y up, X forward, Z to the
// right. Each rotor is mounted so its thrust points along +Y (up)
// regardless of spin direction; only the magnitude matters for lift. The
// torque about Y is yaw; aerodynamic drag opposes each rotor's spin, so
// adjacent rotors must spin in opposite directions to cancel yaw at equal
// speeds.
class QuadcopterForceModel
{
public:
    static constexpr std::size_t kRotorCount = 4;

    explicit QuadcopterForceModel(const QuadcopterParameters& parameters = {});

    // Rotor index must be in [0, 4).
    void setRotorAngularVelocity(std::size_t index, double angular_velocity);
    double rotorAngularVelocity(std::size_t index) const;
    const PropellerModel& rotor(std::size_t index) const;
    math::Vec3 rotorPosition(std::size_t index) const;

    // Net body-frame force: (0, lift, 0) in newtons.
    math::Vec3 force() const;

    // Net body-frame torque in newton-meters. x is pitch, y is yaw, z is
    // roll; a faster front rotor pitches nose-down (negative x).
    math::Vec3 torque() const;

    double totalThrust() const;
    double totalPower() const;

    void reset();

private:
    QuadcopterParameters parameters_;
    std::array<PropellerModel, kRotorCount> rotors_;
};

}  // namespace srp::bridge
