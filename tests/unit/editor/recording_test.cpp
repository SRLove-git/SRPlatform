#include "editor/playback_controller.hpp"
#include "editor/simulation_recorder.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace srp::editor
{
namespace
{

std::vector<BodySnapshot> sampleBodies(double x)
{
    BodySnapshot box;
    box.name = "box_1";
    box.kind = "box";
    box.position = srp::math::Vec3(x, 0.5, 0.0);
    return {box};
}

TEST(SimulationRecorderTest, RecordsOnlyWhileActive)
{
    SimulationRecorder recorder;
    EXPECT_FALSE(recorder.recording());

    recorder.record(0.0, sampleBodies(0.0));
    EXPECT_EQ(recorder.size(), 0u);

    recorder.begin();
    EXPECT_TRUE(recorder.recording());
    recorder.record(0.0, sampleBodies(0.0));
    recorder.record(1.0 / 60.0, sampleBodies(0.1));
    recorder.end();
    EXPECT_FALSE(recorder.recording());
    EXPECT_EQ(recorder.size(), 2u);

    recorder.record(2.0, sampleBodies(0.2));
    EXPECT_EQ(recorder.size(), 2u);
}

TEST(SimulationRecorderTest, ClearResets)
{
    SimulationRecorder recorder;
    recorder.begin();
    recorder.record(0.0, sampleBodies(0.0));
    recorder.clear();
    EXPECT_EQ(recorder.size(), 0u);
    EXPECT_FALSE(recorder.recording());
}

TEST(SimulationRecorderTest, SaveLoadRoundTrip)
{
    SimulationRecorder recorder;
    recorder.begin();
    recorder.record(0.0, sampleBodies(0.0));
    recorder.record(1.0 / 60.0, sampleBodies(1.5));
    recorder.end();

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "srp_recording_roundtrip.json";
    std::string error;
    ASSERT_TRUE(recorder.save(path, error)) << error;

    SimulationRecorder loaded;
    ASSERT_TRUE(loaded.load(path, error)) << error;
    EXPECT_EQ(loaded.size(), 2u);
    EXPECT_EQ(loaded.snapshots().back().bodies.front().position.x, 1.5);
    EXPECT_EQ(loaded.snapshots().front().bodies.front().kind, "box");
    std::filesystem::remove(path);
}

TEST(PlaybackControllerTest, InterpolatesBetweenSnapshots)
{
    PlaybackController playback;
    std::vector<SimSnapshot> snapshots;
    SimSnapshot first;
    first.time_s = 0.0;
    first.bodies = sampleBodies(0.0);
    snapshots.push_back(first);

    SimSnapshot second;
    second.time_s = 2.0;
    second.bodies = sampleBodies(4.0);
    snapshots.push_back(second);

    playback.setSnapshots(std::move(snapshots));
    EXPECT_TRUE(playback.hasSnapshots());
    EXPECT_EQ(playback.duration(), 2.0);

    playback.seek(1.0);
    const auto frame = playback.frame();
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->front().position.x, 2.0);
}

TEST(PlaybackControllerTest, PlayAdvancesAndStopsAtEnd)
{
    PlaybackController playback;
    std::vector<SimSnapshot> snapshots;
    SimSnapshot first;
    first.time_s = 0.0;
    snapshots.push_back(first);
    SimSnapshot second;
    second.time_s = 1.0;
    snapshots.push_back(second);
    playback.setSnapshots(std::move(snapshots));

    EXPECT_FALSE(playback.isPlaying());
    playback.play();
    EXPECT_TRUE(playback.isPlaying());
    playback.advance(0.4);
    EXPECT_NEAR(playback.time(), 0.4, 1e-9);

    playback.pause();
    playback.advance(0.4);
    EXPECT_NEAR(playback.time(), 0.4, 1e-9);

    playback.play();
    playback.advance(1.0);
    EXPECT_TRUE(playback.finished());
    EXPECT_FALSE(playback.isPlaying());

    playback.stop();
    EXPECT_EQ(playback.time(), 0.0);
    EXPECT_FALSE(playback.finished());
}

TEST(PlaybackControllerTest, SeekClampsToDuration)
{
    PlaybackController playback;
    std::vector<SimSnapshot> snapshots;
    SimSnapshot first;
    first.time_s = 0.0;
    snapshots.push_back(first);
    SimSnapshot second;
    second.time_s = 1.0;
    snapshots.push_back(second);
    playback.setSnapshots(std::move(snapshots));

    playback.seek(99.0);
    EXPECT_EQ(playback.time(), 1.0);
    playback.seek(-1.0);
    EXPECT_EQ(playback.time(), 0.0);
}

}  // namespace
}  // namespace srp::editor
