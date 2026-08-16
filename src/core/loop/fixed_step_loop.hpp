#pragma once

#include <chrono>
#include <cstddef>

namespace srp::core
{

class FixedStepLoop
{
public:
    using Clock = std::chrono::steady_clock;

    struct Update
    {
        std::size_t steps{0};
        double alpha{0.0};
    };

    explicit FixedStepLoop(double fixed_dt, std::size_t max_steps_per_frame = 8);

    void reset(Clock::time_point now);
    Update advance(Clock::time_point now);

private:
    double fixed_dt_;
    std::size_t max_steps_per_frame_;
    double accumulator_{0.0};
    Clock::time_point last_time_{};
    bool has_last_time_{false};
};

}  // namespace srp::core
