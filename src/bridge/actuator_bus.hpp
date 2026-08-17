#pragma once

#include "bridge/bridge_types.hpp"

namespace srp::bridge
{

class IActuatorBus
{
public:
    virtual ~IActuatorBus() = default;

    virtual void setMotor(MotorId id, ActuatorValue value) = 0;
    virtual void setServo(ServoId id, ActuatorValue angle) = 0;
};

}  // namespace srp::bridge
