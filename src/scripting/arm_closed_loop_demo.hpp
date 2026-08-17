#pragma once

#include "bridge/arm_entity.hpp"

#include <memory>
#include <string>

namespace srp::scripting
{

class LuaScriptHost;

// Integration object that lets a Lua script drive an example robotic arm
// through the servo actuator bus and read joint angles back as sensors.
class ArmClosedLoopDemo
{
public:
    explicit ArmClosedLoopDemo(
        const bridge::ArmParameters& parameters = {});
    ~ArmClosedLoopDemo();

    ArmClosedLoopDemo(const ArmClosedLoopDemo&) = delete;
    ArmClosedLoopDemo& operator=(const ArmClosedLoopDemo&) = delete;

    bridge::ArmEntity& arm();
    const bridge::ArmEntity& arm() const;

    LuaScriptHost& host();

    bool loadScript(const std::string& id, const std::string& source);
    bool step(double dt);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::scripting
