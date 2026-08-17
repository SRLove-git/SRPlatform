#pragma once

#include "bridge/bridge_types.hpp"

namespace srp::bridge
{

class ISensorBus
{
public:
    virtual ~ISensorBus() = default;

    virtual SensorValue read(SensorId id) const = 0;
};

}  // namespace srp::bridge
