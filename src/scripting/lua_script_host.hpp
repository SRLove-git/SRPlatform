#pragma once

#include "scripting/script_host.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

namespace srp::scripting
{

class LuaScriptHost final : public IScriptHost
{
public:
    LuaScriptHost();
    ~LuaScriptHost() override;

    LuaScriptHost(const LuaScriptHost&) = delete;
    LuaScriptHost& operator=(const LuaScriptHost&) = delete;
    LuaScriptHost(LuaScriptHost&&) = delete;
    LuaScriptHost& operator=(LuaScriptHost&&) = delete;

    bool load(const std::string& id, const std::string& source) override;
    bool reload(const std::string& id) override;
    bool runOnce(double dt) override;

    bool hasScript(const std::string& id) const override;
    std::size_t scriptCount() const override;

    std::optional<std::string> lastError() const;
    std::optional<double> getNumber(
        const std::string& id,
        const std::string& name) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace srp::scripting
