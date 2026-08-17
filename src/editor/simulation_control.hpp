#pragma once

#include <cstddef>
#include <cstdint>

namespace srp::editor
{

enum class SimState
{
    kRunning,
    kPaused
};

// Playback control for the fixed-step simulation loop. While running, the
// loop advances up to max_steps per frame scaled by the speed multiplier;
// while paused it advances nothing unless a manual single step is requested.
class SimulationControl
{
public:
    SimulationControl();

    void play();
    void pause();
    void toggle();
    SimState state() const;
    bool isRunning() const;

    // Queues a single fixed step for the next stepsToRun() call. Only valid
    // while paused; ignored otherwise.
    void requestStep();

    // Number of fixed steps to advance this frame (0 while paused, 1 when a
    // manual step was requested, otherwise scaled max_steps).
    std::size_t stepsToRun(std::size_t max_steps);

    void setSpeed(double multiplier);
    double speed() const;

    void resetCounters();
    std::uint64_t stepsTaken() const;

private:
    SimState state_{SimState::kRunning};
    bool manual_step_requested_{false};
    double speed_{1.0};
    std::uint64_t steps_taken_{0};
};

}  // namespace srp::editor
