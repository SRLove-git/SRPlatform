#include "control/pid_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace srp::control
{

PidController::PidController(const PidGains& gains, const PidLimits& limits)
    : gains_(gains),
      limits_(limits)
{
    if (limits.output_min > limits.output_max)
    {
        throw std::invalid_argument("pid output minimum exceeds maximum");
    }

    if (limits.integral_min > limits.integral_max)
    {
        throw std::invalid_argument("pid integral minimum exceeds maximum");
    }
}

void PidController::setGains(const PidGains& gains)
{
    gains_ = gains;
}

void PidController::setLimits(const PidLimits& limits)
{
    if (limits.output_min > limits.output_max)
    {
        throw std::invalid_argument("pid output minimum exceeds maximum");
    }

    if (limits.integral_min > limits.integral_max)
    {
        throw std::invalid_argument("pid integral minimum exceeds maximum");
    }

    limits_ = limits;
}

double PidController::compute(double error, double dt)
{
    if (dt <= 0.0)
    {
        return output_;
    }

    integral_ = std::clamp(
        integral_ + error * dt,
        limits_.integral_min,
        limits_.integral_max);

    const double derivative =
        has_previous_error_ ? (error - previous_error_) / dt : 0.0;
    previous_error_ = error;
    has_previous_error_ = true;

    output_ = std::clamp(
        gains_.kp * error +
            gains_.ki * integral_ +
            gains_.kd * derivative,
        limits_.output_min,
        limits_.output_max);
    return output_;
}

double PidController::output() const
{
    return output_;
}

double PidController::integral() const
{
    return integral_;
}

void PidController::reset()
{
    integral_ = 0.0;
    previous_error_ = 0.0;
    has_previous_error_ = false;
    output_ = 0.0;
}

}  // namespace srp::control
