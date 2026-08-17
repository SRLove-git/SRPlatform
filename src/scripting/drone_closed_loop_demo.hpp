#pragma once

#include "bridge/drone_entity.hpp"

#include <memory>
#include <string>

namespace srp::scripting
{

class LuaScriptHost;

// Integration object that lets a Lua script hover an example quadcopter
// through the sensor and actuator buses.
class DroneClosedLoopDemo
{
public:
    explicit DroneClosedLoopDemo(
        const bridge::DroneParameters& parameters = {});
    ~DroneClosedLoopDemo();

    DroneClosedLoopDemo(const DroneClosedLoopDemo&) = delete;
    DroneClosedLoopDemo& operator=(const DroneClosedLoopDemo&) = delete;
    DroneClosedLoopDemo(DroneClosedLoopDemo&&) = delete;
    DroneClosedLoopDemo& operator=(DroneClosedLoopDemo&&) = delete;

    bridge::DroneEntity& drone();
    const bridge::DroneEntity& drone() const;

    LuaScriptHost& host();

    bool loadScript(const std::string& id, const std::string& source);
    bool step(double dt);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::scripting
