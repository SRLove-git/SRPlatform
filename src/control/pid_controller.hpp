#pragma once

namespace srp::control
{

// Tuning gains for a PID controller.
struct PidGains
{
    double kp{1.0};
    double ki{0.0};
    double kd{0.0};
};

// Output and integral clamps. Integral clamping prevents windup when the
// error cannot be driven to zero.
struct PidLimits
{
    double output_min{-1.0};
    double output_max{1.0};
    double integral_min{-1.0};
    double integral_max{1.0};
};

// Discrete PID controller with output clamping and integral anti-windup.
//
// compute(error, dt) returns:
//     u = kp * error + ki * integral + kd * derivative
// where integral accumulates error*dt (clamped) and derivative is the
// first-order difference of the error (0 on the first call).
class PidController
{
public:
    PidController(const PidGains& gains = {}, const PidLimits& limits = {});

    void setGains(const PidGains& gains);
    void setLimits(const PidLimits& limits);

    // Advances the controller by one step and returns the clamped output.
    // Non-positive time steps leave the state unchanged.
    double compute(double error, double dt);

    double output() const;
    double integral() const;

    void reset();

private:
    PidGains gains_;
    PidLimits limits_;
    double integral_{0.0};
    double previous_error_{0.0};
    double output_{0.0};
    bool has_previous_error_{false};
};

}  // namespace srp::control
