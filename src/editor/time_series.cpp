#include "editor/time_series.hpp"

#include <algorithm>

namespace srp::editor
{

TimeSeries::TimeSeries(std::size_t capacity)
    : capacity_(std::max<std::size_t>(1, capacity))
{
    samples_.reserve(capacity_);
}

void TimeSeries::record(double time, double value)
{
    samples_.emplace_back(time, value);
    if (samples_.size() > capacity_)
    {
        samples_.erase(
            samples_.begin(),
            samples_.begin() + static_cast<std::ptrdiff_t>(samples_.size() - capacity_));
    }
}

void TimeSeries::clear()
{
    samples_.clear();
}

std::size_t TimeSeries::size() const
{
    return samples_.size();
}

bool TimeSeries::empty() const
{
    return samples_.empty();
}

const std::vector<std::pair<double, double>>& TimeSeries::samples() const
{
    return samples_;
}

std::optional<double> TimeSeries::latestValue() const
{
    if (samples_.empty())
    {
        return std::nullopt;
    }
    return samples_.back().second;
}

std::optional<double> TimeSeries::minimum() const
{
    if (samples_.empty())
    {
        return std::nullopt;
    }
    double minimum = samples_.front().second;
    for (const auto& [time, value] : samples_)
    {
        minimum = std::min(minimum, value);
    }
    return minimum;
}

std::optional<double> TimeSeries::maximum() const
{
    if (samples_.empty())
    {
        return std::nullopt;
    }
    double maximum = samples_.front().second;
    for (const auto& [time, value] : samples_)
    {
        maximum = std::max(maximum, value);
    }
    return maximum;
}

std::optional<double> TimeSeries::latestTime() const
{
    if (samples_.empty())
    {
        return std::nullopt;
    }
    return samples_.back().first;
}

void TimeSeries::setCapacity(std::size_t capacity)
{
    capacity_ = std::max<std::size_t>(1, capacity);
    if (samples_.size() > capacity_)
    {
        samples_.erase(
            samples_.begin(),
            samples_.begin() + static_cast<std::ptrdiff_t>(samples_.size() - capacity_));
    }
}

std::size_t TimeSeries::capacity() const
{
    return capacity_;
}

}  // namespace srp::editor
