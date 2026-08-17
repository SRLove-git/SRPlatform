#include "editor/simulation_control.hpp"

#include <algorithm>

namespace srp::editor
{

SimulationControl::SimulationControl() = default;

void SimulationControl::play()
{
    state_ = SimState::kRunning;
}

void SimulationControl::pause()
{
    state_ = SimState::kPaused;
    manual_step_requested_ = false;
}

void SimulationControl::toggle()
{
    if (state_ == SimState::kRunning)
    {
        pause();
    }
    else
    {
        play();
    }
}

SimState SimulationControl::state() const
{
    return state_;
}

bool SimulationControl::isRunning() const
{
    return state_ == SimState::kRunning;
}

void SimulationControl::requestStep()
{
    if (state_ == SimState::kPaused)
    {
        manual_step_requested_ = true;
    }
}

std::size_t SimulationControl::stepsToRun(std::size_t max_steps)
{
    if (manual_step_requested_)
    {
        manual_step_requested_ = false;
        ++steps_taken_;
        return 1;
    }

    if (state_ != SimState::kRunning)
    {
        return 0;
    }

    std::size_t steps = static_cast<std::size_t>(
        static_cast<double>(max_steps) * speed_);
    if (steps == 0 && speed_ > 0.0)
    {
        steps = 1;
    }
    steps_taken_ += steps;
    return steps;
}

void SimulationControl::setSpeed(double multiplier)
{
    speed_ = std::clamp(multiplier, 0.0, 16.0);
}

double SimulationControl::speed() const
{
    return speed_;
}

void SimulationControl::resetCounters()
{
    steps_taken_ = 0;
}

std::uint64_t SimulationControl::stepsTaken() const
{
    return steps_taken_;
}

}  // namespace srp::editor
