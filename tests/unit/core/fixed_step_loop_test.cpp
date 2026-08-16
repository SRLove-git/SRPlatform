#include "core/loop/fixed_step_loop.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

namespace
{

using Clock = srp::core::FixedStepLoop::Clock;

bool nearlyEqual(double lhs, double rhs, double tolerance = 1e-6)
{
    return std::abs(lhs - rhs) < tolerance;
}

}  // namespace

int main()
{
    constexpr double kFixedDt = 0.01;
    constexpr Clock::duration kFixedDtDuration =
        std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(10'000'000));

    srp::core::FixedStepLoop loop(kFixedDt, 8);

    const Clock::time_point start{Clock::duration{0}};
    const srp::core::FixedStepLoop::Update first = loop.advance(start);
    if (first.steps != 0)
    {
        std::cerr << "first advance should produce zero steps\n";
        return 1;
    }

    const auto half_step = start + kFixedDtDuration / 2;
    const auto half_update = loop.advance(half_step);
    if (half_update.steps != 0 || !nearlyEqual(half_update.alpha, 0.5))
    {
        std::cerr << "half-step accumulator is incorrect\n";
        return 1;
    }

    const auto two_steps = half_step + kFixedDtDuration + kFixedDtDuration / 2;
    const auto two_step_update = loop.advance(two_steps);
    if (two_step_update.steps != 2 || !nearlyEqual(two_step_update.alpha, 0.0))
    {
        std::cerr << "two-step accumulator is incorrect\n";
        return 1;
    }

    const auto long_frame = two_steps + std::chrono::seconds(1);
    const auto long_update = loop.advance(long_frame);
    if (long_update.steps != 8 || !nearlyEqual(long_update.alpha, 0.0))
    {
        std::cerr << "long frame should clamp and reset the accumulator\n";
        return 1;
    }

    return 0;
}
