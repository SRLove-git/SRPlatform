#include "bridge/bridge.hpp"
#include "scripting/lua_script_host.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
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

TEST(ScriptControlApi, LuaCanReadSensorAndControlActuators)
{
    auto actuator_bus = std::make_shared<RecordingActuatorBus>();
    auto sensor_bus = std::make_shared<StubSensorBus>();
    auto bridge = std::make_shared<srp::bridge::Bridge>();
    bridge->attachActuatorBus(actuator_bus);
    bridge->attachSensorBus(sensor_bus);

    srp::scripting::LuaScriptHost host;
    host.bindControl(bridge);

    constexpr const char* script =
        "function update(dt)\n"
        "    local value = read_sensor(5)\n"
        "    set_motor(7, value)\n"
        "    set_servo(9, 1.25)\n"
        "end\n";

    EXPECT_TRUE(host.load("controller", script));
    EXPECT_TRUE(host.runOnce(0.1));

    ASSERT_EQ(actuator_bus->motor_commands.size(), 1U);
    EXPECT_EQ(actuator_bus->motor_commands[0].first, 7U);
    EXPECT_DOUBLE_EQ(actuator_bus->motor_commands[0].second, 2.5);

    ASSERT_EQ(actuator_bus->servo_commands.size(), 1U);
    EXPECT_EQ(actuator_bus->servo_commands[0].first, 9U);
    EXPECT_DOUBLE_EQ(actuator_bus->servo_commands[0].second, 1.25);
}

TEST(ScriptControlApi, ReadSensorReturnsNaNWhenNoBridgeIsBound)
{
    srp::scripting::LuaScriptHost host;

    constexpr const char* script =
        "observed = read_sensor(5)\n"
        "function update(dt)\n"
        "end\n";

    EXPECT_TRUE(host.load("sensor_only", script));
    EXPECT_TRUE(host.runOnce(0.1));

    const auto observed = host.getNumber("sensor_only", "observed");
    ASSERT_TRUE(observed.has_value());
    EXPECT_TRUE(std::isnan(*observed));
}

TEST(ScriptControlApi, SetMotorReturnsFalseWhenNoBridgeIsBound)
{
    srp::scripting::LuaScriptHost host;

    constexpr const char* script =
        "result = set_motor(1, 1.0) and 1 or 0\n"
        "function update(dt)\n"
        "end\n";

    EXPECT_TRUE(host.load("no_bridge", script));
    EXPECT_TRUE(host.runOnce(0.1));

    const auto result = host.getNumber("no_bridge", "result");
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(*result, 0.0);
}
