#include "mod/mod_registry.hpp"

#include <unordered_map>
#include <unordered_set>

namespace srp::mod
{
namespace
{

enum class VisitState
{
    kUnvisited,
    kVisiting,
    kVisited
};

// Depth-first visit that reports missing dependencies and cycles, and
// appends mod ids in dependency-first order (post-order).
bool visit(
    const std::string& id,
    const std::unordered_map<std::string, const ModManifest*>& by_id,
    std::unordered_map<std::string, VisitState>& states,
    std::vector<std::string>& stack,
    std::vector<std::string>& order,
    std::vector<std::string>& errors)
{
    const VisitState state = states[id];
    if (state == VisitState::kVisited)
    {
        return true;
    }
    if (state == VisitState::kVisiting)
    {
        std::string cycle = "dependency cycle detected: ";
        bool in_cycle = false;
        for (const std::string& entry : stack)
        {
            if (entry == id)
            {
                in_cycle = true;
            }
            if (in_cycle)
            {
                cycle += entry + " -> ";
            }
        }
        cycle += id;
        errors.push_back(cycle);
        return false;
    }

    states[id] = VisitState::kVisiting;
    stack.push_back(id);

    const auto manifest_it = by_id.find(id);
    if (manifest_it != by_id.end())
    {
        for (const std::string& dependency : manifest_it->second->dependencies)
        {
            const auto dependency_it = by_id.find(dependency);
            if (dependency_it == by_id.end())
            {
                errors.push_back(
                    "mod '" + id + "' requires missing mod '" +
                    dependency + "'");
                continue;
            }

            if (!visit(dependency, by_id, states, stack, order, errors))
            {
                stack.pop_back();
                return false;
            }
        }
    }

    stack.pop_back();
    states[id] = VisitState::kVisited;
    order.push_back(id);
    return true;
}

}  // namespace

DependencyResolution resolveModDependencies(
    const std::vector<ModManifest>& manifests)
{
    DependencyResolution resolution;
    std::unordered_map<std::string, const ModManifest*> by_id;
    std::unordered_set<std::string> seen_ids;

    for (const ModManifest& manifest : manifests)
    {
        if (manifest.id.empty())
        {
            resolution.errors.push_back("mod manifest has an empty id");
            continue;
        }

        if (!seen_ids.insert(manifest.id).second)
        {
            resolution.errors.push_back(
                "duplicate mod id: '" + manifest.id + "'");
            continue;
        }
        by_id[manifest.id] = &manifest;
    }

    if (!resolution.errors.empty())
    {
        return resolution;
    }

    std::unordered_map<std::string, VisitState> states;
    for (const ModManifest& manifest : manifests)
    {
        states[manifest.id] = VisitState::kUnvisited;
    }

    std::vector<std::string> stack;
    for (const ModManifest& manifest : manifests)
    {
        visit(
            manifest.id,
            by_id,
            states,
            stack,
            resolution.load_order,
            resolution.errors);
    }

    if (!resolution.errors.empty())
    {
        resolution.ok = false;
        resolution.load_order.clear();
        return resolution;
    }

    resolution.ok = true;
    return resolution;
}

}  // namespace srp::mod
