#pragma once

#include <cstdint>

namespace srp::bridge
{

// Actuator and sensor handles are stable identifiers, mirroring the ID
// conventions used by the physics and circuit modules.
using ActuatorId = std::uint32_t;
using MotorId = ActuatorId;
using ServoId = ActuatorId;
using SensorId = std::uint32_t;

constexpr ActuatorId kInvalidActuatorId = 0;
constexpr MotorId kInvalidMotorId = kInvalidActuatorId;
constexpr ServoId kInvalidServoId = kInvalidActuatorId;
constexpr SensorId kInvalidSensorId = 0;

// Buses use SI-compatible double precision values. Actuator commands are
// deliberately untyped at this layer because motor and servo semantics will be
// defined by the concrete bridge implementations in later Phase 3 tasks.
using ActuatorValue = double;
using SensorValue = double;

}  // namespace srp::bridge
