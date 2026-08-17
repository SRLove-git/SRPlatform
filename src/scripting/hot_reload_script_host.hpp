#pragma once

#include "scripting/lua_script_host.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace srp::bridge
{
class Bridge;
}

namespace srp::scripting
{

// File-backed Lua script host with hot reload.
//
// Scripts are loaded from files by id. pollReloads() compares each file's
// last-write time and size with the previous load and recompiles the script
// when the file changed, so mod authors can edit scripts without restarting
// the simulation.
class HotReloadScriptHost
{
public:
    HotReloadScriptHost();
    ~HotReloadScriptHost();

    HotReloadScriptHost(const HotReloadScriptHost&) = delete;
    HotReloadScriptHost& operator=(const HotReloadScriptHost&) = delete;

    bool loadFromFile(
        const std::string& id,
        const std::filesystem::path& path);
    bool reload(const std::string& id);

    // Rechecks every tracked file and reloads changed scripts. Returns true
    // when at least one script was reloaded.
    bool pollReloads();

    bool runOnce(double dt);
    bool hasScript(const std::string& id) const;

    void bindControl(std::shared_ptr<bridge::Bridge> bridge);
    std::optional<std::string> lastError() const;

    LuaScriptHost& host();
    const LuaScriptHost& host() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::scripting
