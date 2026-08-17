#include "bridge/bridge.hpp"

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace
{

class RecordingActuatorBus final : public srp::bridge::IActuatorBus
{
public:
    void setMotor(srp::bridge::MotorId id, srp::bridge::ActuatorValue value) override
    {
        motor_commands.emplace_back(id, value);
    }

    void setServo(srp::bridge::ServoId id, srp::bridge::ActuatorValue angle) override
    {
        servo_commands.emplace_back(id, angle);
    }

    std::vector<std::pair<srp::bridge::MotorId, srp::bridge::ActuatorValue>> motor_commands;
    std::vector<std::pair<srp::bridge::ServoId, srp::bridge::ActuatorValue>> servo_commands;
};

class StubSensorBus final : public srp::bridge::ISensorBus
{
public:
    srp::bridge::SensorValue read(srp::bridge::SensorId id) const override
    {
        return static_cast<srp::bridge::SensorValue>(id) * 0.5;
    }
};

}  // namespace

TEST(BridgeActuatorBus, IsImplementableThroughInterface)
{
    RecordingActuatorBus bus;
    srp::bridge::IActuatorBus& actuator_bus = bus;

    actuator_bus.setMotor(3, 0.75);
    actuator_bus.setServo(5, 1.25);

    ASSERT_EQ(bus.motor_commands.size(), 1U);
    EXPECT_EQ(bus.motor_commands[0].first, 3U);
    EXPECT_DOUBLE_EQ(bus.motor_commands[0].second, 0.75);

    ASSERT_EQ(bus.servo_commands.size(), 1U);
    EXPECT_EQ(bus.servo_commands[0].first, 5U);
    EXPECT_DOUBLE_EQ(bus.servo_commands[0].second, 1.25);
}

TEST(BridgeSensorBus, IsImplementableThroughInterface)
{
    StubSensorBus bus;
    const srp::bridge::ISensorBus& sensor_bus = bus;

    EXPECT_DOUBLE_EQ(sensor_bus.read(8), 4.0);
}

TEST(Bridge, ForwardsActuatorCommandsToAttachedBus)
{
    auto bus = std::make_shared<RecordingActuatorBus>();
    srp::bridge::Bridge bridge;
    bridge.attachActuatorBus(bus);

    EXPECT_TRUE(bridge.hasActuatorBus());
    EXPECT_TRUE(bridge.setMotor(11, -0.25));
    EXPECT_TRUE(bridge.setServo(13, 0.5));

    ASSERT_EQ(bus->motor_commands.size(), 1U);
    EXPECT_EQ(bus->motor_commands[0].first, 11U);
    EXPECT_DOUBLE_EQ(bus->motor_commands[0].second, -0.25);

    ASSERT_EQ(bus->servo_commands.size(), 1U);
    EXPECT_EQ(bus->servo_commands[0].first, 13U);
    EXPECT_DOUBLE_EQ(bus->servo_commands[0].second, 0.5);
}

TEST(Bridge, ReadsSensorFromAttachedBus)
{
    auto bus = std::make_shared<StubSensorBus>();
    srp::bridge::Bridge bridge;
    bridge.attachSensorBus(bus);

    EXPECT_TRUE(bridge.hasSensorBus());

    const auto value = bridge.readSensor(10);
    ASSERT_TRUE(value.has_value());
    EXPECT_DOUBLE_EQ(*value, 5.0);
}

TEST(Bridge, RejectsCommandsWhenNoBusIsAttached)
{
    srp::bridge::Bridge bridge;

    EXPECT_FALSE(bridge.hasActuatorBus());
    EXPECT_FALSE(bridge.hasSensorBus());
    EXPECT_FALSE(bridge.setMotor(1, 1.0));
    EXPECT_FALSE(bridge.setServo(1, 1.0));
    EXPECT_FALSE(bridge.readSensor(1).has_value());
}

TEST(Bridge, DetachingBusRemovesConnection)
{
    auto bus = std::make_shared<RecordingActuatorBus>();
    srp::bridge::Bridge bridge;
    bridge.attachActuatorBus(bus);
    bridge.detachActuatorBus();

    EXPECT_FALSE(bridge.hasActuatorBus());
    EXPECT_FALSE(bridge.setMotor(1, 1.0));
    EXPECT_TRUE(bus->motor_commands.empty());
}
