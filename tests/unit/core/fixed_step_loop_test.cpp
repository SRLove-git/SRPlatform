#include "core/loop/fixed_step_loop.hpp"

#include <chrono>

#include <gtest/gtest.h>

TEST(CoreFixedStepLoop, FirstAdvanceReturnsZeroSteps)
{
    using Clock = srp::core::FixedStepLoop::Clock;

    srp::core::FixedStepLoop loop(0.01, 8);
    const auto first = loop.advance(Clock::time_point{Clock::duration{0}});
    EXPECT_EQ(first.steps, 0);
}

TEST(CoreFixedStepLoop, HalfStepProducesCorrectAlpha)
{
    using Clock = srp::core::FixedStepLoop::Clock;

    constexpr Clock::duration kFixedDt =
        std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(10'000'000));

    srp::core::FixedStepLoop loop(0.01, 8);
    const Clock::time_point start{Clock::duration{0}};
    loop.advance(start);

    const auto half_step = start + kFixedDt / 2;
    const auto half_update = loop.advance(half_step);
    EXPECT_EQ(half_update.steps, 0);
    EXPECT_NEAR(half_update.alpha, 0.5, 1e-9);
}

TEST(CoreFixedStepLoop, MultipleStepsAccumulateCorrectly)
{
    using Clock = srp::core::FixedStepLoop::Clock;

    constexpr Clock::duration kFixedDt =
        std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(10'000'000));

    srp::core::FixedStepLoop loop(0.01, 8);
    const Clock::time_point start{Clock::duration{0}};
    loop.advance(start);

    const auto half_step = start + kFixedDt / 2;
    loop.advance(half_step);
    const auto two_steps = half_step + kFixedDt + kFixedDt / 2;
    const auto two_step_update = loop.advance(two_steps);
    EXPECT_EQ(two_step_update.steps, 2);
    EXPECT_NEAR(two_step_update.alpha, 0.0, 1e-9);
}

TEST(CoreFixedStepLoop, LongFrameClampsAndResets)
{
    using Clock = srp::core::FixedStepLoop::Clock;

    constexpr Clock::duration kFixedDt =
        std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(10'000'000));

    srp::core::FixedStepLoop loop(0.01, 8);
    const Clock::time_point start{Clock::duration{0}};
    loop.advance(start);

    const auto half_step = start + kFixedDt / 2;
    loop.advance(half_step);
    const auto two_steps = half_step + kFixedDt + kFixedDt / 2;
    loop.advance(two_steps);
    const auto long_frame = two_steps + std::chrono::seconds(1);
    const auto long_update = loop.advance(long_frame);
    EXPECT_EQ(long_update.steps, 8);
    EXPECT_NEAR(long_update.alpha, 0.0, 1e-9);
}
