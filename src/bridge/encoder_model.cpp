#include "bridge/encoder_model.hpp"

#include <cmath>
#include <stdexcept>

namespace srp::bridge
{
namespace
{

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

}  // namespace

EncoderModel::EncoderModel(const EncoderParameters& parameters)
    : parameters_(parameters)
{
    if (parameters.counts_per_revolution == 0)
    {
        throw std::invalid_argument(
            "encoder counts per revolution must be positive");
    }
}

void EncoderModel::update(
    double shaft_angle_rad,
    double shaft_angular_velocity_rad_s)
{
    angle_rad_ = shaft_angle_rad;
    angular_velocity_rad_s_ = shaft_angular_velocity_rad_s;
}

double EncoderModel::angleRad() const
{
    return angle_rad_;
}

double EncoderModel::angularVelocityRadS() const
{
    return angular_velocity_rad_s_;
}

std::int64_t EncoderModel::counts() const
{
    const double counts_per_rad =
        static_cast<double>(parameters_.counts_per_revolution) / kTwoPi;
    return static_cast<std::int64_t>(std::floor(angle_rad_ * counts_per_rad));
}

double EncoderModel::countsPerSecond() const
{
    const double counts_per_rad =
        static_cast<double>(parameters_.counts_per_revolution) / kTwoPi;
    return angular_velocity_rad_s_ * counts_per_rad;
}

void EncoderModel::reset()
{
    angle_rad_ = 0.0;
    angular_velocity_rad_s_ = 0.0;
}

}  // namespace srp::bridge
