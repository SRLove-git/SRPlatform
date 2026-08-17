#include "scripting/hot_reload_script_host.hpp"

#include "bridge/bridge.hpp"

#include <fstream>
#include <sstream>
#include <unordered_map>

namespace srp::scripting
{

struct HotReloadScriptHost::Impl
{
    struct TrackedFile
    {
        std::filesystem::path path;
        std::filesystem::file_time_type last_write_time;
        std::uintmax_t size{0};
    };

    LuaScriptHost host;
    std::unordered_map<std::string, TrackedFile> files;
    std::string last_error;

    bool readFile(const std::filesystem::path& path, std::string& source)
    {
        std::ifstream stream(path);
        if (!stream.is_open())
        {
            return false;
        }

        std::ostringstream buffer;
        buffer << stream.rdbuf();
        source = buffer.str();
        return true;
    }

    bool refreshStats(TrackedFile& file)
    {
        std::error_code ec;
        const std::filesystem::file_time_type write_time =
            std::filesystem::last_write_time(file.path, ec);
        const std::uintmax_t size =
            std::filesystem::file_size(file.path, ec);
        if (ec)
        {
            return false;
        }

        file.last_write_time = write_time;
        file.size = size;
        return true;
    }
};

HotReloadScriptHost::HotReloadScriptHost()
    : impl_(std::make_unique<Impl>())
{
}

HotReloadScriptHost::~HotReloadScriptHost() = default;

bool HotReloadScriptHost::loadFromFile(
    const std::string& id,
    const std::filesystem::path& path)
{
    impl_->last_error.clear();

    std::string source;
    if (!impl_->readFile(path, source))
    {
        impl_->last_error = "cannot open script file: " + path.string();
        return false;
    }

    if (!impl_->host.load(id, source))
    {
        impl_->last_error = impl_->host.lastError().value_or(
            "failed to load script");
        return false;
    }

    Impl::TrackedFile file;
    file.path = path;
    if (!impl_->refreshStats(file))
    {
        impl_->last_error = "cannot stat script file: " + path.string();
        return false;
    }

    impl_->files[id] = std::move(file);
    return true;
}

bool HotReloadScriptHost::reload(const std::string& id)
{
    impl_->last_error.clear();

    const auto it = impl_->files.find(id);
    if (it == impl_->files.end())
    {
        impl_->last_error = "script not found: " + id;
        return false;
    }

    std::string source;
    if (!impl_->readFile(it->second.path, source))
    {
        impl_->last_error =
            "cannot open script file: " + it->second.path.string();
        return false;
    }

    if (!impl_->host.load(id, source))
    {
        impl_->last_error = impl_->host.lastError().value_or(
            "failed to reload script");
        return false;
    }

    impl_->refreshStats(it->second);
    return true;
}

bool HotReloadScriptHost::pollReloads()
{
    bool reloaded = false;

    for (auto& [id, file] : impl_->files)
    {
        std::error_code ec;
        const std::filesystem::file_time_type write_time =
            std::filesystem::last_write_time(file.path, ec);
        const std::uintmax_t size =
            std::filesystem::file_size(file.path, ec);
        if (ec)
        {
            continue;
        }

        if (write_time != file.last_write_time || size != file.size)
        {
            if (reload(id))
            {
                reloaded = true;
            }
        }
    }

    return reloaded;
}

bool HotReloadScriptHost::runOnce(double dt)
{
    const bool ok = impl_->host.runOnce(dt);
    if (!ok)
    {
        impl_->last_error =
            impl_->host.lastError().value_or("script runtime error");
    }
    return ok;
}

bool HotReloadScriptHost::hasScript(const std::string& id) const
{
    return impl_->files.find(id) != impl_->files.end();
}

void HotReloadScriptHost::bindControl(
    std::shared_ptr<bridge::Bridge> bridge)
{
    impl_->host.bindControl(std::move(bridge));
}

std::optional<std::string> HotReloadScriptHost::lastError() const
{
    if (impl_->last_error.empty())
    {
        return std::nullopt;
    }
    return impl_->last_error;
}

LuaScriptHost& HotReloadScriptHost::host()
{
    return impl_->host;
}

const LuaScriptHost& HotReloadScriptHost::host() const
{
    return impl_->host;
}

}  // namespace srp::scripting
