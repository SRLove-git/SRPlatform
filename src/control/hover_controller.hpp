#pragma once

#include "control/pid_controller.hpp"

namespace srp::control
{

// Tuning gains for the cascaded altitude/velocity hover controller.
struct HoverGains
{
    // Outer loop: altitude error -> target vertical velocity.
    double altitude_kp{2.0};
    double altitude_ki{0.5};
    double altitude_kd{0.8};

    // Inner loop: velocity error -> throttle adjustment. The default uses
    // PI here; the outer loop's derivative already provides damping, and a
    // derivative on the velocity error tends to chatter at the output clamp.
    double velocity_kp{1.5};
    double velocity_ki{0.3};
    double velocity_kd{0.0};
};

struct HoverLimits
{
    // Clamp for the outer loop's target vertical velocity, m/s.
    double max_vertical_velocity_m_s{2.0};

    // Normalized throttle command range.
    double throttle_min{0.0};
    double throttle_max{1.0};

    // Feedforward throttle that balances gravity at hover. Tune this to the
    // craft's thrust-to-weight ratio.
    double gravity_compensation_throttle{0.5};
};

// Example altitude-hold controller for a multirotor.
//
// A cascaded PID keeps the craft at a target altitude:
//
//     altitude PID -> target vertical velocity
//     velocity PID -> throttle adjustment
//     throttle = gravity_compensation + velocity PID output
//
// The returned throttle is normalized and clamped to the configured range.
// Users can treat this class as a template for their own hover controller.
class HoverController
{
public:
    HoverController(const HoverGains& gains = {}, const HoverLimits& limits = {});

    // Returns the normalized throttle command in [throttle_min, throttle_max].
    double update(
        double altitude_m,
        double vertical_velocity_m_s,
        double target_altitude_m,
        double dt);

    double verticalVelocityCommand() const;
    double throttle() const;

    void reset();

private:
    PidController altitude_pid_;
    PidController velocity_pid_;
    HoverLimits limits_;
    double vertical_velocity_command_{0.0};
    double throttle_{0.0};
};

}  // namespace srp::control
