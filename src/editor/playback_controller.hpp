#pragma once

#include "editor/simulation_recorder.hpp"

#include <optional>
#include <vector>

namespace srp::editor
{

// Playback of recorded snapshots with linear position and slerp orientation
// interpolation between the bracketing frames.
class PlaybackController
{
public:
    void setSnapshots(std::vector<SimSnapshot> snapshots);
    bool hasSnapshots() const;

    void play();
    void pause();
    void stop();
    bool isPlaying() const;

    void seek(double time);
    double time() const;
    double duration() const;
    bool finished() const;

    // Advances the playback clock by dt (clamped to the recording duration).
    void advance(double dt);

    // Interpolated frame at the current playback time. Empty when there are
    // no snapshots or playback finished.
    std::optional<std::vector<BodySnapshot>> frame() const;

private:
    std::vector<SimSnapshot> snapshots_;
    bool playing_{false};
    double time_{0.0};
    double duration_{0.0};
};

}  // namespace srp::editor
