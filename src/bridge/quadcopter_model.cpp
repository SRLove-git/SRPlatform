#include "bridge/quadcopter_model.hpp"

#include <cmath>
#include <stdexcept>

namespace srp::bridge
{
namespace
{

// Rotor positions in the body frame for the "+" layout.
constexpr std::array<math::Vec3, QuadcopterForceModel::kRotorCount> kRotorOffsets = {
    math::Vec3(1.0, 0.0, 0.0),   // front
    math::Vec3(0.0, 0.0, -1.0),  // right
    math::Vec3(-1.0, 0.0, 0.0),  // back
    math::Vec3(0.0, 0.0, 1.0)    // left
};

// Spin direction about the +Y axis for each rotor. Adjacent rotors spin in
// opposite directions so that equal speeds cancel aerodynamic yaw torque.
constexpr std::array<double, QuadcopterForceModel::kRotorCount> kSpinDirection = {
    1.0,   // front
    -1.0,  // right
    1.0,   // back
    -1.0   // left
};

void checkRotorIndex(std::size_t index)
{
    if (index >= QuadcopterForceModel::kRotorCount)
    {
        throw std::out_of_range("rotor index out of range");
    }
}

}  // namespace

QuadcopterForceModel::QuadcopterForceModel(const QuadcopterParameters& parameters)
    : parameters_(parameters)
{
    if (parameters.arm_length_m <= 0.0)
    {
        throw std::invalid_argument("quadcopter arm length must be positive");
    }

    for (std::size_t i = 0; i < kRotorCount; ++i)
    {
        rotors_[i] = PropellerModel(parameters.propeller);
    }
}

void QuadcopterForceModel::setRotorAngularVelocity(
    std::size_t index,
    double angular_velocity)
{
    checkRotorIndex(index);
    rotors_[index].setAngularVelocity(kSpinDirection[index] * angular_velocity);
}

double QuadcopterForceModel::rotorAngularVelocity(std::size_t index) const
{
    checkRotorIndex(index);
    return rotors_[index].angularVelocity();
}

const PropellerModel& QuadcopterForceModel::rotor(std::size_t index) const
{
    checkRotorIndex(index);
    return rotors_[index];
}

math::Vec3 QuadcopterForceModel::rotorPosition(std::size_t index) const
{
    checkRotorIndex(index);
    return parameters_.arm_length_m * kRotorOffsets[index];
}

math::Vec3 QuadcopterForceModel::force() const
{
    return math::Vec3(0.0, totalThrust(), 0.0);
}

math::Vec3 QuadcopterForceModel::torque() const
{
    // Moment from a thrust force at a rotor position: M = r x F with
    // F = (0, T, 0), giving M_x = -z*T and M_z = x*T.
    double pitch_torque = 0.0;
    double yaw_torque = 0.0;
    double roll_torque = 0.0;

    for (std::size_t i = 0; i < kRotorCount; ++i)
    {
        const math::Vec3 offset = kRotorOffsets[i];
        // Rotors are mounted so that all four always push upward; the spin
        // direction only affects the aerodynamic yaw torque.
        const double thrust = std::abs(rotors_[i].thrust());
        pitch_torque += -offset.z * thrust;
        roll_torque += offset.x * thrust;
        yaw_torque += rotors_[i].torque();
    }

    const double arm = parameters_.arm_length_m;
    return math::Vec3(arm * pitch_torque, yaw_torque, arm * roll_torque);
}

double QuadcopterForceModel::totalThrust() const
{
    double thrust = 0.0;
    for (const PropellerModel& rotor : rotors_)
    {
        thrust += std::abs(rotor.thrust());
    }
    return thrust;
}

double QuadcopterForceModel::totalPower() const
{
    double power = 0.0;
    for (const PropellerModel& rotor : rotors_)
    {
        power += rotor.power();
    }
    return power;
}

void QuadcopterForceModel::reset()
{
    for (PropellerModel& rotor : rotors_)
    {
        rotor.reset();
    }
}

}  // namespace srp::bridge
