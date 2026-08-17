#pragma once

#include "bridge/actuator_bus.hpp"
#include "bridge/sensor_bus.hpp"

#include <memory>
#include <optional>

namespace srp::bridge
{

// Bridge is the attachment point between the simulation core and the
// electromechanical conversion layer. Concrete actuator and sensor buses can
// be swapped without changing callers.
class Bridge
{
public:
    void attachActuatorBus(std::shared_ptr<IActuatorBus> bus);
    void attachSensorBus(std::shared_ptr<ISensorBus> bus);

    void detachActuatorBus();
    void detachSensorBus();

    bool hasActuatorBus() const;
    bool hasSensorBus() const;

    bool setMotor(MotorId id, ActuatorValue value);
    bool setServo(ServoId id, ActuatorValue angle);

    std::optional<SensorValue> readSensor(SensorId id) const;

private:
    std::shared_ptr<IActuatorBus> actuator_bus_;
    std::shared_ptr<ISensorBus> sensor_bus_;
};

}  // namespace srp::bridge
