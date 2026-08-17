#pragma once

#include "bridge/car_entity.hpp"

#include <memory>
#include <string>

namespace srp::scripting
{

class LuaScriptHost;

// Small integration object that lets a Lua script control an example car
// through the sensor and actuator buses.
class CarClosedLoopDemo
{
public:
    explicit CarClosedLoopDemo(
        const bridge::CarParameters& parameters = {});
    ~CarClosedLoopDemo();

    CarClosedLoopDemo(const CarClosedLoopDemo&) = delete;
    CarClosedLoopDemo& operator=(const CarClosedLoopDemo&) = delete;
    CarClosedLoopDemo(CarClosedLoopDemo&&) = delete;
    CarClosedLoopDemo& operator=(CarClosedLoopDemo&&) = delete;

    bridge::CarEntity& car();
    const bridge::CarEntity& car() const;

    LuaScriptHost& host();

    bool loadScript(const std::string& id, const std::string& source);
    bool step(double dt);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::scripting
