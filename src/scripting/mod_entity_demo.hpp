#pragma once

#include "bridge/entity.hpp"
#include "scripting/hot_reload_script_host.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace srp::scripting
{

// Loads a mod directory, instantiates its blueprint entity, and runs its
// entry script against the entity through the standard actuator/sensor
// buses. Script changes are picked up by pollReloads().
class ModEntityDemo
{
public:
    ModEntityDemo();
    ~ModEntityDemo();

    ModEntityDemo(const ModEntityDemo&) = delete;
    ModEntityDemo& operator=(const ModEntityDemo&) = delete;

    // Loads mod.json, the entity blueprint (manifest "blueprint" field or
    // "blueprint.json"), and the entry script.
    bool load(const std::filesystem::path& mod_directory, std::string& error);

    bool step(double dt);
    bool pollReloads();

    bridge::IEntity* entity();
    const bridge::IEntity* entity() const;
    HotReloadScriptHost& scripts();
    const HotReloadScriptHost& scripts() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::scripting
