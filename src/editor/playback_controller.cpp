#include "editor/playback_controller.hpp"

#include <algorithm>

#include <glm/gtc/quaternion.hpp>

namespace srp::editor
{

void PlaybackController::setSnapshots(std::vector<SimSnapshot> snapshots)
{
    snapshots_ = std::move(snapshots);
    duration_ = snapshots_.empty() ? 0.0 : snapshots_.back().time_s;
    time_ = 0.0;
    playing_ = false;
}

bool PlaybackController::hasSnapshots() const
{
    return snapshots_.size() >= 2;
}

void PlaybackController::play()
{
    if (hasSnapshots() && !finished())
    {
        playing_ = true;
    }
}

void PlaybackController::pause()
{
    playing_ = false;
}

void PlaybackController::stop()
{
    playing_ = false;
    time_ = 0.0;
}

bool PlaybackController::isPlaying() const
{
    return playing_;
}

void PlaybackController::seek(double time)
{
    time_ = std::clamp(time, 0.0, duration_);
}

double PlaybackController::time() const
{
    return time_;
}

double PlaybackController::duration() const
{
    return duration_;
}

bool PlaybackController::finished() const
{
    return !snapshots_.empty() && time_ >= duration_;
}

void PlaybackController::advance(double dt)
{
    if (!playing_)
    {
        return;
    }
    time_ = std::min(time_ + dt, duration_);
    if (finished())
    {
        playing_ = false;
    }
}

std::optional<std::vector<BodySnapshot>> PlaybackController::frame() const
{
    if (snapshots_.empty() || time_ > duration_)
    {
        return std::nullopt;
    }

    if (time_ <= snapshots_.front().time_s)
    {
        return snapshots_.front().bodies;
    }
    if (time_ >= snapshots_.back().time_s)
    {
        return snapshots_.back().bodies;
    }

    std::size_t next_index = 1;
    while (next_index + 1 < snapshots_.size() &&
           snapshots_[next_index].time_s < time_)
    {
        ++next_index;
    }

    const SimSnapshot& before = snapshots_[next_index - 1];
    const SimSnapshot& after = snapshots_[next_index];
    const double span = after.time_s - before.time_s;
    const double alpha = span > 1e-12
        ? (time_ - before.time_s) / span
        : 0.0;

    const std::size_t count = std::min(
        before.bodies.size(),
        after.bodies.size());
    std::vector<BodySnapshot> interpolated;
    interpolated.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        BodySnapshot body;
        body.name = before.bodies[i].name;
        body.kind = before.bodies[i].kind;
        body.position = glm::mix(
            before.bodies[i].position,
            after.bodies[i].position,
            alpha);
        body.orientation = glm::slerp(
            before.bodies[i].orientation,
            after.bodies[i].orientation,
            alpha);
        interpolated.push_back(std::move(body));
    }
    return interpolated;
}

}  // namespace srp::editor
