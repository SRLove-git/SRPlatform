#pragma once

#include "bridge/entity.hpp"
#include "mod/entity_blueprint.hpp"

#include <memory>
#include <string>

namespace srp::bridge
{

// Instantiates the entity described by a blueprint. Supported kinds are
// "car" and "drone"; blueprint parameters override the defaults of the
// corresponding entity parameters.
std::unique_ptr<IEntity> createEntity(
    const mod::EntityBlueprint& blueprint,
    std::string& error);

}  // namespace srp::bridge
