#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace srp::editor
{

// Fixed-capacity time series of (time, value) samples used by the
// observation panel. Older samples fall off when the capacity is exceeded.
class TimeSeries
{
public:
    explicit TimeSeries(std::size_t capacity = 1200);

    void record(double time, double value);
    void clear();

    std::size_t size() const;
    bool empty() const;

    const std::vector<std::pair<double, double>>& samples() const;

    std::optional<double> latestValue() const;
    std::optional<double> minimum() const;
    std::optional<double> maximum() const;
    std::optional<double> latestTime() const;

    void setCapacity(std::size_t capacity);
    std::size_t capacity() const;

private:
    std::vector<std::pair<double, double>> samples_;
    std::size_t capacity_;
};

}  // namespace srp::editor
