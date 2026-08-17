#include "bridge/state_recorder.hpp"

namespace srp::bridge
{

void StateRecorder::record(const CarStateSample& sample)
{
    samples_.push_back(sample);
}

void StateRecorder::clear()
{
    samples_.clear();
}

std::size_t StateRecorder::size() const
{
    return samples_.size();
}

const std::vector<CarStateSample>& StateRecorder::samples() const
{
    return samples_;
}

std::optional<CarStateSample> StateRecorder::latest() const
{
    if (samples_.empty())
    {
        return std::nullopt;
    }

    return samples_.back();
}

}  // namespace srp::bridge
