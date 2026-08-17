#include "editor/simulation_control.hpp"

#include <gtest/gtest.h>

namespace srp::editor
{
namespace
{

TEST(SimulationControlTest, StartsRunning)
{
    SimulationControl control;
    EXPECT_TRUE(control.isRunning());
    EXPECT_EQ(control.state(), SimState::kRunning);
}

TEST(SimulationControlTest, PauseAndPlay)
{
    SimulationControl control;
    control.pause();
    EXPECT_FALSE(control.isRunning());
    EXPECT_EQ(control.stepsToRun(8), 0u);

    control.play();
    EXPECT_TRUE(control.isRunning());
    EXPECT_EQ(control.stepsToRun(8), 8u);
}

TEST(SimulationControlTest, ToggleFlipsState)
{
    SimulationControl control;
    control.toggle();
    EXPECT_FALSE(control.isRunning());
    control.toggle();
    EXPECT_TRUE(control.isRunning());
}

TEST(SimulationControlTest, ManualStepRunsExactlyOnce)
{
    SimulationControl control;
    control.pause();
    control.requestStep();

    EXPECT_EQ(control.stepsToRun(8), 1u);
    // The queued step is consumed; the next call advances nothing.
    EXPECT_EQ(control.stepsToRun(8), 0u);
}

TEST(SimulationControlTest, ManualStepIgnoredWhileRunning)
{
    SimulationControl control;
    control.requestStep();
    EXPECT_EQ(control.stepsToRun(8), 8u);
}

TEST(SimulationControlTest, SpeedScalesSteps)
{
    SimulationControl control;
    control.setSpeed(0.5);
    EXPECT_EQ(control.stepsToRun(8), 4u);

    control.setSpeed(2.0);
    EXPECT_EQ(control.stepsToRun(3), 6u);
}

TEST(SimulationControlTest, SpeedIsClamped)
{
    SimulationControl control;
    control.setSpeed(-1.0);
    EXPECT_EQ(control.speed(), 0.0);
    control.setSpeed(100.0);
    EXPECT_EQ(control.speed(), 16.0);
}

TEST(SimulationControlTest, StepCounterTracksProgress)
{
    SimulationControl control;
    control.pause();
    control.requestStep();
    control.stepsToRun(8);
    EXPECT_EQ(control.stepsTaken(), 1u);

    control.play();
    control.stepsToRun(5);
    EXPECT_EQ(control.stepsTaken(), 6u);

    control.resetCounters();
    EXPECT_EQ(control.stepsTaken(), 0u);
}

}  // namespace
}  // namespace srp::editor
