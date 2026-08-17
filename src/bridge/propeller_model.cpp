#include "bridge/propeller_model.hpp"

#include <cmath>
#include <stdexcept>

namespace srp::bridge
{
namespace
{

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

double revolutionsPerSecond(double angular_velocity)
{
    return angular_velocity / kTwoPi;
}

}  // namespace

PropellerModel::PropellerModel(const PropellerParameters& parameters)
    : parameters_(parameters)
{
    if (parameters.diameter_m <= 0.0)
    {
        throw std::invalid_argument("propeller diameter must be positive");
    }

    if (parameters.thrust_coefficient <= 0.0)
    {
        throw std::invalid_argument("propeller thrust coefficient must be positive");
    }

    if (parameters.torque_coefficient <= 0.0)
    {
        throw std::invalid_argument("propeller torque coefficient must be positive");
    }

    if (parameters.air_density_kg_m3 <= 0.0)
    {
        throw std::invalid_argument("propeller air density must be positive");
    }
}

double PropellerModel::angularVelocity() const
{
    return angular_velocity_rad_s_;
}

double PropellerModel::thrust() const
{
    const double n = revolutionsPerSecond(angular_velocity_rad_s_);
    const double diameter = parameters_.diameter_m;
    const double diameter_4 = diameter * diameter * diameter * diameter;
    return parameters_.thrust_coefficient *
        parameters_.air_density_kg_m3 *
        n * std::abs(n) *
        diameter_4;
}

double PropellerModel::torque() const
{
    const double n = revolutionsPerSecond(angular_velocity_rad_s_);
    const double diameter = parameters_.diameter_m;
    const double diameter_5 = diameter * diameter * diameter * diameter * diameter;
    return -parameters_.torque_coefficient *
        parameters_.air_density_kg_m3 *
        n * std::abs(n) *
        diameter_5;
}

double PropellerModel::power() const
{
    const double n = revolutionsPerSecond(angular_velocity_rad_s_);
    const double diameter = parameters_.diameter_m;
    const double diameter_5 = diameter * diameter * diameter * diameter * diameter;
    return parameters_.torque_coefficient *
        parameters_.air_density_kg_m3 *
        std::abs(n * n * n) * kTwoPi *
        diameter_5;
}

void PropellerModel::setAngularVelocity(double angular_velocity)
{
    angular_velocity_rad_s_ = angular_velocity;
}

void PropellerModel::reset()
{
    angular_velocity_rad_s_ = 0.0;
}

}  // namespace srp::bridge
