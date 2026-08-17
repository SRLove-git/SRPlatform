#include "editor/sim_observer.hpp"
#include "editor/time_series.hpp"

#include <gtest/gtest.h>

namespace srp::editor
{
namespace
{

TEST(TimeSeriesTest, RecordsAndKeepsCapacity)
{
    TimeSeries series(4);
    for (int i = 0; i < 10; ++i)
    {
        series.record(static_cast<double>(i), static_cast<double>(i) * 2.0);
    }

    ASSERT_EQ(series.size(), 4u);
    EXPECT_EQ(series.samples().front().first, 6.0);
    EXPECT_EQ(series.samples().back().first, 9.0);
    EXPECT_EQ(series.latestValue(), 18.0);
    EXPECT_EQ(series.minimum(), 12.0);
    EXPECT_EQ(series.maximum(), 18.0);
}

TEST(TimeSeriesTest, ClearAndEmptyBehavior)
{
    TimeSeries series(8);
    EXPECT_TRUE(series.empty());
    EXPECT_FALSE(series.latestValue().has_value());
    EXPECT_FALSE(series.minimum().has_value());
    EXPECT_FALSE(series.maximum().has_value());

    series.record(0.0, 5.0);
    EXPECT_FALSE(series.empty());
    series.clear();
    EXPECT_TRUE(series.empty());
}

TEST(TimeSeriesTest, SetCapacityTrimsOldSamples)
{
    TimeSeries series(100);
    for (int i = 0; i < 10; ++i)
    {
        series.record(static_cast<double>(i), static_cast<double>(i));
    }

    series.setCapacity(4);
    ASSERT_EQ(series.size(), 4u);
    EXPECT_EQ(series.samples().front().first, 6.0);
}

TEST(SimObserverTest, RecordsChannelsWithTime)
{
    SimObserver observer;
    observer.setSimTime(1.0);
    observer.record("voltage", 3.7);
    observer.record("voltage", 3.8);
    observer.recordSensor("front", 0.4);

    EXPECT_TRUE(observer.hasChannel("voltage"));
    EXPECT_TRUE(observer.hasChannel("sensor_front"));
    EXPECT_EQ(observer.channel("voltage").size(), 2u);
    EXPECT_EQ(observer.channel("voltage").latestValue(), 3.8);
    EXPECT_EQ(observer.channel("sensor_front").latestValue(), 0.4);
    EXPECT_EQ(observer.channelNames().size(), 2u);
}

TEST(SimObserverTest, AccumulateIntegratesPower)
{
    SimObserver observer;
    observer.setSimTime(0.0);
    observer.accumulate("energy_j", 10.0, 0.5);   // 5 J
    observer.accumulate("energy_j", 10.0, 0.5);   // 10 J

    EXPECT_EQ(observer.channel("energy_j").latestValue(), 10.0);
    EXPECT_EQ(observer.channel("energy_j").minimum(), 5.0);
}

TEST(SimObserverTest, ClearDropsChannels)
{
    SimObserver observer;
    observer.record("a", 1.0);
    observer.clear();
    EXPECT_FALSE(observer.hasChannel("a"));
    EXPECT_TRUE(observer.channelNames().empty());
}

TEST(SimObserverTest, DroneTelemetryRecordsAltitude)
{
    srp::bridge::DroneEntity drone;
    SimObserver observer;
    sampleDroneTelemetry(drone, observer);

    EXPECT_TRUE(observer.hasChannel("drone_altitude_m"));
    EXPECT_TRUE(observer.hasChannel("drone_vertical_velocity_m_s"));
    EXPECT_GE(observer.channel("drone_altitude_m").latestValue().value_or(-1.0), 0.0);
}

TEST(SimObserverTest, ArmTelemetryRecordsJointsAndEffector)
{
    srp::bridge::ArmEntity arm;
    arm.setServo(0, 1.0);
    for (int i = 0; i < 60; ++i)
    {
        arm.step(1.0 / 60.0);
    }

    SimObserver observer;
    sampleArmTelemetry(arm, observer);

    EXPECT_TRUE(observer.hasChannel("arm_joint_1_rad"));
    EXPECT_TRUE(observer.hasChannel("arm_joint_2_rad"));
    EXPECT_TRUE(observer.hasChannel("arm_effector_x_m"));
    EXPECT_GT(observer.channel("arm_joint_1_rad").latestValue().value_or(0.0), 0.5);
}

}  // namespace
}  // namespace srp::editor
