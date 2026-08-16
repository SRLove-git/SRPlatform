#include "core/loop/fixed_step_loop.hpp"

namespace srp::core
{

FixedStepLoop::FixedStepLoop(double fixed_dt, std::size_t max_steps_per_frame)
    : fixed_dt_(fixed_dt),
      max_steps_per_frame_(max_steps_per_frame)
{
}

void FixedStepLoop::reset(Clock::time_point now)
{
    last_time_ = now;
    accumulator_ = 0.0;
    has_last_time_ = true;
}

FixedStepLoop::Update FixedStepLoop::advance(Clock::time_point now)
{
    if (!has_last_time_)
    {
        last_time_ = now;
        has_last_time_ = true;
        return {};
    }

    const auto elapsed = now - last_time_;
    last_time_ = now;

    double frame_time = std::chrono::duration<double>(elapsed).count();
    if (frame_time < 0.0)
    {
        frame_time = 0.0;
    }

    accumulator_ += frame_time;

    std::size_t steps = 0;
    while (accumulator_ >= fixed_dt_ && steps < max_steps_per_frame_)
    {
        accumulator_ -= fixed_dt_;
        ++steps;
    }

    if (steps == max_steps_per_frame_ && accumulator_ >= fixed_dt_)
    {
        accumulator_ = 0.0;
    }

    return {steps, accumulator_ / fixed_dt_};
}

}  // namespace srp::core
