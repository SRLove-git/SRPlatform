#pragma once

#include "core/math/types.hpp"
#include "physics/physics_world.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace srp::editor
{

enum class ShapeKind
{
    kBox,
    kSphere,
    kCylinder,
    kPlane
};

const char* shapeKindName(ShapeKind kind);

struct BodyEntry
{
    std::string name;
    srp::physics::BodyId id{srp::physics::kInvalidBodyId};
    ShapeKind kind{ShapeKind::kBox};
};

// Editable sandbox scene. Owns the physics world and keeps a stable, named
// list of user-created bodies on top of a static ground plane.
class SceneEditor
{
public:
    SceneEditor();

    // Creates a dynamic primitive at the given position (or at a deterministic
    // spread position when omitted) and returns its body id.
    srp::physics::BodyId addBody(
        ShapeKind kind,
        const srp::math::Vec3& position);
    srp::physics::BodyId addBody(ShapeKind kind);

    // Removes a body (and joints referencing it). Returns false when the id is
    // unknown or is the ground plane.
    bool removeBody(srp::physics::BodyId id);

    // Removes every user-created object, keeping only the ground.
    void clearObjects();

    bool select(srp::physics::BodyId id);
    void deselect();
    std::optional<srp::physics::BodyId> selected() const;

    BodyEntry* entry(srp::physics::BodyId id);
    const BodyEntry* entry(srp::physics::BodyId id) const;
    const std::vector<BodyEntry>& entries() const;

    // Teleports a body and resets its velocity. Returns false for unknown ids.
    bool moveTo(srp::physics::BodyId id, const srp::math::Vec3& position);

    srp::physics::PhysicsWorld& world();
    const srp::physics::PhysicsWorld& world() const;

    srp::physics::BodyId groundBody() const;

    static srp::physics::CollisionShape shapeFor(ShapeKind kind);
    static double spawnHeight(ShapeKind kind);

    // Casts a ray against every non-ground body and returns the nearest hit.
    static std::optional<srp::physics::BodyId> pick(
        const srp::physics::PhysicsWorld& world,
        const srp::math::Vec3& ray_origin,
        const srp::math::Vec3& ray_direction,
        double max_distance = 1000.0);

    // Intersects a ray with the ground plane (y = 0) and returns the hit point.
    static std::optional<srp::math::Vec3> pickGround(
        const srp::math::Vec3& ray_origin,
        const srp::math::Vec3& ray_direction);

private:
    std::string nextName(ShapeKind kind);

    srp::physics::PhysicsWorld world_;
    std::vector<BodyEntry> entries_;
    std::optional<srp::physics::BodyId> selected_;
    std::size_t next_index_{1};
    srp::physics::BodyId ground_body_{srp::physics::kInvalidBodyId};
};

}  // namespace srp::editor
