#pragma once

#include <cstddef>
#include <string>

namespace srp::scripting
{

class IScriptHost
{
public:
    virtual ~IScriptHost() = default;

    virtual bool load(const std::string& id, const std::string& source) = 0;
    virtual bool reload(const std::string& id) = 0;
    virtual bool runOnce(double dt) = 0;

    virtual bool hasScript(const std::string& id) const = 0;
    virtual std::size_t scriptCount() const = 0;
};

}  // namespace srp::scripting
