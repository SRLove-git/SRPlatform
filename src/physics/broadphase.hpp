#pragma once

#include "physics/aabb.hpp"
#include "physics/rigid_body.hpp"

#include <unordered_map>
#include <vector>

namespace srp::physics
{

struct BodyPair
{
    BodyId first{kInvalidBodyId};
    BodyId second{kInvalidBodyId};
};

class Broadphase
{
public:
    void upsert(BodyId id, const Aabb& aabb);
    void remove(BodyId id);
    std::vector<BodyPair> findOverlappingPairs() const;

private:
    std::unordered_map<BodyId, Aabb> entries_;
};

}  // namespace srp::physics
