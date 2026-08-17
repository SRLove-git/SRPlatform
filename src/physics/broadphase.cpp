#include "physics/broadphase.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace srp::physics
{

void Broadphase::upsert(BodyId id, const Aabb& aabb)
{
    entries_[id] = aabb;
}

void Broadphase::remove(BodyId id)
{
    entries_.erase(id);
}

std::vector<BodyPair> Broadphase::findOverlappingPairs() const
{
    std::set<std::pair<BodyId, BodyId>> unique_pairs;

    std::vector<std::pair<BodyId, Aabb>> finite_entries;
    std::vector<std::pair<BodyId, Aabb>> infinite_entries;

    for (const auto& [id, aabb] : entries_)
    {
        if (aabb.is_finite)
        {
            finite_entries.emplace_back(id, aabb);
        }
        else
        {
            infinite_entries.emplace_back(id, aabb);
        }
    }

    for (const auto& infinite : infinite_entries)
    {
        for (const auto& other : entries_)
        {
            if (infinite.first == other.first)
            {
                continue;
            }

            const auto pair = std::minmax(infinite.first, other.first);
            unique_pairs.insert(pair);
        }
    }

    std::sort(
        finite_entries.begin(),
        finite_entries.end(),
        [](const auto& lhs, const auto& rhs)
        {
            return lhs.second.min.x < rhs.second.min.x;
        });

    for (std::size_t i = 0; i < finite_entries.size(); ++i)
    {
        for (std::size_t j = i + 1; j < finite_entries.size(); ++j)
        {
            if (finite_entries[j].second.min.x > finite_entries[i].second.max.x)
            {
                break;
            }

            if (overlaps(finite_entries[i].second, finite_entries[j].second))
            {
                unique_pairs.insert(std::minmax(
                    finite_entries[i].first,
                    finite_entries[j].first));
            }
        }
    }

    std::vector<BodyPair> result;
    result.reserve(unique_pairs.size());

    for (const auto& [first, second] : unique_pairs)
    {
        result.push_back({first, second});
    }

    return result;
}

}  // namespace srp::physics
