#include "bridge/distance_sensor_model.hpp"

#include "physics/collision_shape.hpp"
#include "physics/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <variant>

#include <glm/gtc/quaternion.hpp>

namespace srp::bridge
{
namespace
{

constexpr double kEpsilon = 1e-12;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

std::optional<double> rayBox(
    const math::Vec3& origin,
    const math::Vec3& direction,
    const physics::BoxShape& box,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    const math::Quat inverse = glm::inverse(orientation);
    const math::Vec3 local_origin = inverse * (origin - position);
    const math::Vec3 local_direction = inverse * direction;

    double t_min = 0.0;
    double t_max = kInfinity;

    for (int axis = 0; axis < 3; ++axis)
    {
        const double center = local_origin[axis];
        const double direction_axis = local_direction[axis];
        const double half_extent = box.half_extents[axis];

        if (std::abs(direction_axis) < kEpsilon)
        {
            if (center < -half_extent || center > half_extent)
            {
                return std::nullopt;
            }
            continue;
        }

        double t1 = (-half_extent - center) / direction_axis;
        double t2 = (half_extent - center) / direction_axis;
        if (t1 > t2)
        {
            std::swap(t1, t2);
        }
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        if (t_min > t_max)
        {
            return std::nullopt;
        }
    }

    if (t_max < 0.0)
    {
        return std::nullopt;
    }
    return std::max(t_min, 0.0);
}

std::optional<double> raySphere(
    const math::Vec3& origin,
    const math::Vec3& direction,
    const math::Vec3& center,
    double radius)
{
    const math::Vec3 delta = origin - center;
    const double b = glm::dot(direction, delta);
    const double c = glm::dot(delta, delta) - radius * radius;
    const double discriminant = b * b - c;

    if (discriminant < 0.0)
    {
        return std::nullopt;
    }

    const double root = std::sqrt(discriminant);
    const double t1 = -b - root;
    const double t2 = -b + root;
    if (t2 < 0.0)
    {
        return std::nullopt;
    }
    return std::max(t1, 0.0);
}

std::optional<double> raySphereShape(
    const math::Vec3& origin,
    const math::Vec3& direction,
    const physics::SphereShape& sphere,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    return raySphere(origin, direction, position, sphere.radius);
}

std::optional<double> rayPlane(
    const math::Vec3& origin,
    const math::Vec3& direction,
    const physics::PlaneShape& plane,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    const math::Vec3 normal = glm::normalize(orientation * plane.normal);
    const math::Vec3 point = position + normal * plane.offset;
    const double denominator = glm::dot(direction, normal);

    if (std::abs(denominator) < kEpsilon)
    {
        return std::nullopt;
    }

    const double t = glm::dot(point - origin, normal) / denominator;
    if (t < 0.0)
    {
        return std::nullopt;
    }
    return t;
}

std::optional<double> rayCylinder(
    const math::Vec3& origin,
    const math::Vec3& direction,
    const physics::CylinderShape& cylinder,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    const math::Quat inverse = glm::inverse(orientation);
    const math::Vec3 local_origin = inverse * (origin - position);
    const math::Vec3 local_direction = inverse * direction;
    const double radius = cylinder.radius;
    const double half_height = cylinder.half_height;
    double best = kInfinity;

    const double side_a =
        local_direction.x * local_direction.x +
        local_direction.z * local_direction.z;
    if (side_a > kEpsilon)
    {
        const double side_b = 2.0 * (
            local_origin.x * local_direction.x +
            local_origin.z * local_direction.z);
        const double side_c =
            local_origin.x * local_origin.x +
            local_origin.z * local_origin.z -
            radius * radius;
        const double discriminant = side_b * side_b - 4.0 * side_a * side_c;
        if (discriminant >= 0.0)
        {
            const double root = std::sqrt(discriminant);
            const double t1 = (-side_b - root) / (2.0 * side_a);
            const double t2 = (-side_b + root) / (2.0 * side_a);

            for (const double t : {t1, t2})
            {
                if (t >= 0.0 &&
                    std::abs(local_origin.y + t * local_direction.y) <=
                        half_height)
                {
                    best = std::min(best, t);
                }
            }
        }
    }

    if (std::abs(local_direction.y) > kEpsilon)
    {
        for (const double cap_y : {-half_height, half_height})
        {
            const double t = (cap_y - local_origin.y) / local_direction.y;
            if (t >= 0.0)
            {
                const double x = local_origin.x + t * local_direction.x;
                const double z = local_origin.z + t * local_direction.z;
                if (x * x + z * z <= radius * radius + kEpsilon)
                {
                    best = std::min(best, t);
                }
            }
        }
    }

    if (best == kInfinity)
    {
        return std::nullopt;
    }
    return best;
}

std::optional<double> rayConvexHull(
    const math::Vec3& origin,
    const math::Vec3& direction,
    const physics::ConvexHullShape& hull,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    if (hull.points.empty())
    {
        return std::nullopt;
    }

    math::Vec3 center{0.0};
    for (const math::Vec3& point : hull.points)
    {
        center += point;
    }
    center /= static_cast<double>(hull.points.size());

    double radius = 0.0;
    for (const math::Vec3& point : hull.points)
    {
        radius = std::max(radius, glm::length(point - center));
    }

    const math::Quat inverse = glm::inverse(orientation);
    const math::Vec3 local_origin = inverse * (origin - position);
    const math::Vec3 local_direction = inverse * direction;
    return raySphere(local_origin, local_direction, center, radius);
}

std::optional<double> rayShape(
    const math::Vec3& origin,
    const math::Vec3& direction,
    const physics::CollisionShape& shape,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    return std::visit(
        [&](const auto& value) -> std::optional<double>
        {
            using T = std::decay_t<decltype(value)>;

            if constexpr (std::is_same_v<T, physics::BoxShape>)
            {
                return rayBox(origin, direction, value, position, orientation);
            }
            else if constexpr (std::is_same_v<T, physics::SphereShape>)
            {
                return raySphereShape(origin, direction, value, position, orientation);
            }
            else if constexpr (std::is_same_v<T, physics::PlaneShape>)
            {
                return rayPlane(origin, direction, value, position, orientation);
            }
            else if constexpr (std::is_same_v<T, physics::CylinderShape>)
            {
                return rayCylinder(origin, direction, value, position, orientation);
            }
            else
            {
                return rayConvexHull(origin, direction, value, position, orientation);
            }
        },
        shape);
}

}  // namespace

DistanceSensorModel::DistanceSensorModel(
    const DistanceSensorParameters& parameters)
    : parameters_(parameters)
{
    if (parameters.max_range_m <= 0.0)
    {
        throw std::invalid_argument(
            "distance sensor max range must be positive");
    }

    const double beam_length = glm::length(parameters.beam_axis_local);
    if (beam_length <= kEpsilon)
    {
        throw std::invalid_argument(
            "distance sensor beam axis must be non-zero");
    }
    parameters_.beam_axis_local /= beam_length;

    // Identity pose at construction: the beam points along the local axis.
    beam_direction_world_ = parameters_.beam_axis_local;
    distance_m_ = parameters_.max_range_m;
}

void DistanceSensorModel::setPose(
    const math::Vec3& position,
    const math::Quat& orientation)
{
    position_ = position;
    orientation_ = glm::normalize(orientation);
    beam_direction_world_ =
        glm::normalize(orientation_ * parameters_.beam_axis_local);
}

math::Vec3 DistanceSensorModel::origin() const
{
    return position_;
}

math::Vec3 DistanceSensorModel::beamDirectionWorld() const
{
    return beam_direction_world_;
}

void DistanceSensorModel::setIgnoredBody(physics::BodyId id)
{
    ignored_body_ = id;
}

void DistanceSensorModel::update(const physics::PhysicsWorld& world)
{
    double nearest = parameters_.max_range_m;
    bool hit = false;

    for (const physics::BodyId id : world.bodyIds())
    {
        if (id == ignored_body_)
        {
            continue;
        }

        const physics::RigidBodyState* body = world.body(id);
        const physics::CollisionShape* shape = world.shape(id);
        if (body == nullptr || shape == nullptr)
        {
            continue;
        }

        const std::optional<double> t = rayShape(
            position_,
            beam_direction_world_,
            *shape,
            body->position,
            body->orientation);
        if (t.has_value() && *t <= parameters_.max_range_m && *t < nearest)
        {
            nearest = *t;
            hit = true;
        }
    }

    detected_ = hit;
    distance_m_ = nearest;
}

bool DistanceSensorModel::detected() const
{
    return detected_;
}

double DistanceSensorModel::distance() const
{
    return distance_m_;
}

void DistanceSensorModel::reset()
{
    detected_ = false;
    distance_m_ = parameters_.max_range_m;
    beam_direction_world_ =
        glm::normalize(orientation_ * parameters_.beam_axis_local);
}

}  // namespace srp::bridge
