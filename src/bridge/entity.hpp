#pragma once

namespace srp::bridge
{

// Common interface for simulatable entities (vehicles, robots, ...).
class IEntity
{
public:
    virtual ~IEntity() = default;

    virtual void step(double dt) = 0;

    // Stable kind name used by blueprints, e.g. "car" or "drone".
    virtual const char* kind() const = 0;
};

}  // namespace srp::bridge
