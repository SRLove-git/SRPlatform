#include "control/hover_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace srp::control
{

HoverController::HoverController(
    const HoverGains& gains,
    const HoverLimits& limits)
    : altitude_pid_(
          PidGains{gains.altitude_kp, gains.altitude_ki, gains.altitude_kd},
          PidLimits{
              -limits.max_vertical_velocity_m_s,
              limits.max_vertical_velocity_m_s,
              -limits.max_vertical_velocity_m_s,
              limits.max_vertical_velocity_m_s}),
      velocity_pid_(
          PidGains{gains.velocity_kp, gains.velocity_ki, gains.velocity_kd},
          PidLimits{
              limits.throttle_min - limits.gravity_compensation_throttle,
              limits.throttle_max - limits.gravity_compensation_throttle,
              -1.0,
              1.0}),
      limits_(limits)
{
    if (limits.max_vertical_velocity_m_s <= 0.0)
    {
        throw std::invalid_argument(
            "hover max vertical velocity must be positive");
    }

    if (limits.throttle_min > limits.throttle_max)
    {
        throw std::invalid_argument(
            "hover throttle minimum exceeds maximum");
    }
}

double HoverController::update(
    double altitude_m,
    double vertical_velocity_m_s,
    double target_altitude_m,
    double dt)
{
    vertical_velocity_command_ = altitude_pid_.compute(
        target_altitude_m - altitude_m,
        dt);

    const double throttle_adjustment = velocity_pid_.compute(
        vertical_velocity_command_ - vertical_velocity_m_s,
        dt);

    throttle_ = std::clamp(
        limits_.gravity_compensation_throttle + throttle_adjustment,
        limits_.throttle_min,
        limits_.throttle_max);
    return throttle_;
}

double HoverController::verticalVelocityCommand() const
{
    return vertical_velocity_command_;
}

double HoverController::throttle() const
{
    return throttle_;
}

void HoverController::reset()
{
    altitude_pid_.reset();
    velocity_pid_.reset();
    vertical_velocity_command_ = 0.0;
    throttle_ = 0.0;
}

}  // namespace srp::control
