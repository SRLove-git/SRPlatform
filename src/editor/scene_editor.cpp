#include "editor/scene_editor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/gtc/quaternion.hpp>

namespace srp::editor
{

namespace
{

constexpr double kBoxHalfExtent = 0.5;
constexpr double kSphereRadius = 0.5;
constexpr double kCylinderHalfHeight = 0.5;
constexpr double kCylinderRadius = 0.3;

double rayBox(
    const srp::math::Vec3& origin,
    const srp::math::Vec3& direction,
    const srp::math::Vec3& half_extents)
{
    constexpr double kInfinity = 1e300;
    double t_min = -kInfinity;
    double t_max = kInfinity;

    for (int axis = 0; axis < 3; ++axis)
    {
        const double o = origin[axis];
        const double d = direction[axis];
        const double h = half_extents[axis];

        if (std::abs(d) < 1e-12)
        {
            if (o < -h || o > h)
            {
                return -1.0;
            }
            continue;
        }

        double t1 = (-h - o) / d;
        double t2 = (h - o) / d;
        if (t1 > t2)
        {
            std::swap(t1, t2);
        }
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        if (t_max < t_min)
        {
            return -1.0;
        }
    }

    if (t_max < 0.0)
    {
        return -1.0;
    }
    return t_min >= 0.0 ? t_min : t_max;
}

double raySphere(
    const srp::math::Vec3& origin,
    const srp::math::Vec3& direction,
    double radius)
{
    const double b = glm::dot(origin, direction);
    const double c = glm::dot(origin, origin) - radius * radius;
    const double discriminant = b * b - c;
    if (discriminant < 0.0)
    {
        return -1.0;
    }

    const double sqrt_discriminant = std::sqrt(discriminant);
    double t = -b - sqrt_discriminant;
    if (t < 0.0)
    {
        t = -b + sqrt_discriminant;
    }
    return t >= 0.0 ? t : -1.0;
}

double rayCylinder(
    const srp::math::Vec3& origin,
    const srp::math::Vec3& direction,
    double radius,
    double half_height)
{
    double best_t = -1.0;

    const double a = direction.x * direction.x + direction.z * direction.z;
    const double b = 2.0 * (origin.x * direction.x + origin.z * direction.z);
    const double c = origin.x * origin.x + origin.z * origin.z - radius * radius;

    if (std::abs(a) > 1e-12)
    {
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0.0)
        {
            const double sqrt_discriminant = std::sqrt(discriminant);
            const double candidates[2] = {
                (-b - sqrt_discriminant) / (2.0 * a),
                (-b + sqrt_discriminant) / (2.0 * a)};
            for (const double t : candidates)
            {
                if (t > 0.0)
                {
                    const double y = origin.y + t * direction.y;
                    if (y >= -half_height && y <= half_height)
                    {
                        best_t = best_t < 0.0 ? t : std::min(best_t, t);
                    }
                }
            }
        }
    }

    for (const double cap_y : {-half_height, half_height})
    {
        if (std::abs(direction.y) < 1e-12)
        {
            continue;
        }
        const double t = (cap_y - origin.y) / direction.y;
        if (t <= 0.0)
        {
            continue;
        }
        const double x = origin.x + t * direction.x;
        const double z = origin.z + t * direction.z;
        if (x * x + z * z <= radius * radius + 1e-9)
        {
            best_t = best_t < 0.0 ? t : std::min(best_t, t);
        }
    }

    return best_t;
}

srp::math::Vec3 localOrigin(
    const srp::math::Vec3& world_origin,
    const srp::physics::RigidBodyState& body)
{
    const srp::math::Mat3 rotation = glm::mat3_cast(body.orientation);
    return glm::transpose(rotation) * (world_origin - body.position);
}

srp::math::Vec3 localDirection(
    const srp::math::Vec3& world_direction,
    const srp::physics::RigidBodyState& body)
{
    const srp::math::Mat3 rotation = glm::mat3_cast(body.orientation);
    return glm::transpose(rotation) * world_direction;
}

}  // namespace

const char* shapeKindName(ShapeKind kind)
{
    switch (kind)
    {
    case ShapeKind::kBox:
        return "box";
    case ShapeKind::kSphere:
        return "sphere";
    case ShapeKind::kCylinder:
        return "cylinder";
    case ShapeKind::kPlane:
        return "plane";
    }
    return "unknown";
}

SceneEditor::SceneEditor()
{
    srp::physics::RigidBodyState ground_state;
    ground_state.type = srp::physics::RigidBodyType::kStatic;

    srp::physics::PlaneShape ground_plane;
    ground_plane.normal = srp::math::Vec3(0.0, 1.0, 0.0);
    ground_body_ = world_.createBody(ground_state, ground_plane);
}

srp::physics::CollisionShape SceneEditor::shapeFor(ShapeKind kind)
{
    switch (kind)
    {
    case ShapeKind::kBox:
    {
        srp::physics::BoxShape shape;
        shape.half_extents = srp::math::Vec3(kBoxHalfExtent);
        return shape;
    }
    case ShapeKind::kSphere:
    {
        srp::physics::SphereShape shape;
        shape.radius = kSphereRadius;
        return shape;
    }
    case ShapeKind::kCylinder:
    {
        srp::physics::CylinderShape shape;
        shape.half_height = kCylinderHalfHeight;
        shape.radius = kCylinderRadius;
        return shape;
    }
    case ShapeKind::kPlane:
    {
        srp::physics::PlaneShape shape;
        shape.normal = srp::math::Vec3(0.0, 1.0, 0.0);
        return shape;
    }
    }
    return srp::physics::BoxShape{};
}

double SceneEditor::spawnHeight(ShapeKind kind)
{
    switch (kind)
    {
    case ShapeKind::kBox:
        return kBoxHalfExtent;
    case ShapeKind::kSphere:
        return kSphereRadius;
    case ShapeKind::kCylinder:
        return kCylinderHalfHeight;
    case ShapeKind::kPlane:
        return 0.0;
    }
    return kBoxHalfExtent;
}

std::string SceneEditor::nextName(ShapeKind kind)
{
    return std::string(shapeKindName(kind)) + "_" + std::to_string(next_index_++);
}

srp::physics::BodyId SceneEditor::addBody(
    ShapeKind kind,
    const srp::math::Vec3& position)
{
    srp::physics::RigidBodyState state;
    state.type = srp::physics::RigidBodyType::kDynamic;
    state.mass = 1.0;
    state.friction = 0.6;
    state.restitution = 0.1;
    state.position = position;

    const srp::physics::BodyId id =
        world_.createBody(state, shapeFor(kind));

    BodyEntry entry;
    entry.name = nextName(kind);
    entry.id = id;
    entry.kind = kind;
    entries_.push_back(entry);
    return id;
}

srp::physics::BodyId SceneEditor::addBody(ShapeKind kind)
{
    const std::size_t object_count = entries_.size();
    const double x = (static_cast<double>(object_count % 5) - 2.0) * 1.2;
    const double z = (static_cast<double>((object_count / 5) % 3) - 1.0) * 1.2;
    return addBody(
        kind,
        srp::math::Vec3(x, spawnHeight(kind), z));
}

bool SceneEditor::removeBody(srp::physics::BodyId id)
{
    if (id == ground_body_)
    {
        return false;
    }

    const auto entry_it = std::find_if(
        entries_.begin(),
        entries_.end(),
        [id](const BodyEntry& entry)
        {
            return entry.id == id;
        });
    if (entry_it == entries_.end())
    {
        return false;
    }

    if (selected_ == id)
    {
        selected_.reset();
    }
    entries_.erase(entry_it);
    return world_.removeBody(id);
}

void SceneEditor::clearObjects()
{
    std::vector<srp::physics::BodyId> ids;
    ids.reserve(entries_.size());
    for (const BodyEntry& entry : entries_)
    {
        ids.push_back(entry.id);
    }
    for (const srp::physics::BodyId id : ids)
    {
        removeBody(id);
    }
}

bool SceneEditor::select(srp::physics::BodyId id)
{
    if (id != ground_body_ && world_.body(id) != nullptr)
    {
        selected_ = id;
        return true;
    }
    return false;
}

void SceneEditor::deselect()
{
    selected_.reset();
}

std::optional<srp::physics::BodyId> SceneEditor::selected() const
{
    return selected_;
}

BodyEntry* SceneEditor::entry(srp::physics::BodyId id)
{
    const auto it = std::find_if(
        entries_.begin(),
        entries_.end(),
        [id](const BodyEntry& entry)
        {
            return entry.id == id;
        });
    return it == entries_.end() ? nullptr : &(*it);
}

const BodyEntry* SceneEditor::entry(srp::physics::BodyId id) const
{
    const auto it = std::find_if(
        entries_.begin(),
        entries_.end(),
        [id](const BodyEntry& entry)
        {
            return entry.id == id;
        });
    return it == entries_.end() ? nullptr : &(*it);
}

const std::vector<BodyEntry>& SceneEditor::entries() const
{
    return entries_;
}

bool SceneEditor::moveTo(
    srp::physics::BodyId id,
    const srp::math::Vec3& position)
{
    srp::physics::RigidBodyState* body = world_.body(id);
    if (body == nullptr || id == ground_body_)
    {
        return false;
    }
    body->position = position;
    body->linear_velocity = srp::math::Vec3(0.0);
    body->angular_velocity = srp::math::Vec3(0.0);
    return true;
}

srp::physics::PhysicsWorld& SceneEditor::world()
{
    return world_;
}

const srp::physics::PhysicsWorld& SceneEditor::world() const
{
    return world_;
}

srp::physics::BodyId SceneEditor::groundBody() const
{
    return ground_body_;
}

std::optional<srp::physics::BodyId> SceneEditor::pick(
    const srp::physics::PhysicsWorld& world,
    const srp::math::Vec3& ray_origin,
    const srp::math::Vec3& ray_direction,
    double max_distance)
{
    const srp::math::Vec3 normalized_direction = glm::normalize(ray_direction);
    double best_t = max_distance;
    srp::physics::BodyId best_id = srp::physics::kInvalidBodyId;

    for (const srp::physics::BodyId id : world.bodyIds())
    {
        const srp::physics::RigidBodyState* body = world.body(id);
        const srp::physics::CollisionShape* shape = world.shape(id);
        if (body == nullptr || shape == nullptr)
        {
            continue;
        }

        const srp::math::Vec3 local_origin = localOrigin(ray_origin, *body);
        const srp::math::Vec3 local_direction = localDirection(normalized_direction, *body);

        double t = -1.0;
        std::visit(
            [&](const auto& value)
            {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, srp::physics::BoxShape>)
                {
                    t = rayBox(local_origin, local_direction, value.half_extents);
                }
                else if constexpr (std::is_same_v<T, srp::physics::SphereShape>)
                {
                    t = raySphere(local_origin, local_direction, value.radius);
                }
                else if constexpr (std::is_same_v<T, srp::physics::CylinderShape>)
                {
                    t = rayCylinder(
                        local_origin,
                        local_direction,
                        value.radius,
                        value.half_height);
                }
            },
            *shape);

        if (t > 0.0 && t < best_t)
        {
            best_t = t;
            best_id = id;
        }
    }

    if (best_id == srp::physics::kInvalidBodyId)
    {
        return std::nullopt;
    }
    return best_id;
}

std::optional<srp::math::Vec3> SceneEditor::pickGround(
    const srp::math::Vec3& ray_origin,
    const srp::math::Vec3& ray_direction)
{
    if (std::abs(ray_direction.y) < 1e-12)
    {
        return std::nullopt;
    }
    const double t = -ray_origin.y / ray_direction.y;
    if (t <= 0.0)
    {
        return std::nullopt;
    }
    return ray_origin + ray_direction * t;
}

}  // namespace srp::editor
