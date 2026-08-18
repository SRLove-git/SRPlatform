#include "physics/narrowphase.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

namespace srp::physics
{

namespace
{

math::Vec3 clampVector(
    const math::Vec3& value,
    const math::Vec3& minimum,
    const math::Vec3& maximum)
{
    return {
        std::clamp(value.x, minimum.x, maximum.x),
        std::clamp(value.y, minimum.y, maximum.y),
        std::clamp(value.z, minimum.z, maximum.z)};
}

math::Vec3 worldHalfExtent(const math::Quat& orientation, const math::Vec3& half_extents)
{
    const math::Mat3 rotation = glm::mat3_cast(orientation);
    math::Vec3 result{};

    for (int column = 0; column < 3; ++column)
    {
        result[column] =
            std::abs(rotation[column][0]) * half_extents.x +
            std::abs(rotation[column][1]) * half_extents.y +
            std::abs(rotation[column][2]) * half_extents.z;
    }

    return result;
}

math::Vec3 planeNormal(const PlaneShape& plane, const math::Quat& orientation)
{
    return glm::normalize(orientation * plane.normal);
}

math::Vec3 planePoint(
    const PlaneShape& plane,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    return position + planeNormal(plane, orientation) * plane.offset;
}

bool collideSphereSphere(
    const SphereShape& sphere_a,
    const math::Vec3& position_a,
    const SphereShape& sphere_b,
    const math::Vec3& position_b,
    CollisionResult& result)
{
    const math::Vec3 delta = position_b - position_a;
    const double distance_squared = glm::dot(delta, delta);
    const double radius_sum = sphere_a.radius + sphere_b.radius;

    if (distance_squared > radius_sum * radius_sum)
    {
        return false;
    }

    const double distance = std::sqrt(distance_squared);
    const math::Vec3 normal =
        distance > 1e-12 ? delta / distance : math::Vec3(0.0, 1.0, 0.0);

    result.collided = true;
    result.normal = normal;
    result.penetration = radius_sum - distance;
    result.point = position_a + normal * (sphere_a.radius - result.penetration * 0.5);
    return true;
}

bool collideSpherePlane(
    const SphereShape& sphere,
    const math::Vec3& sphere_position,
    const PlaneShape& plane,
    const math::Vec3& plane_position,
    const math::Quat& plane_orientation,
    CollisionResult& result)
{
    const math::Vec3 normal = planeNormal(plane, plane_orientation);
    const math::Vec3 point = planePoint(plane, plane_position, plane_orientation);
    const double distance = glm::dot(sphere_position - point, normal);

    if (distance >= sphere.radius)
    {
        return false;
    }

    result.collided = true;
    result.normal = -normal;
    result.penetration = sphere.radius - distance;
    result.point = sphere_position - normal * distance;
    return true;
}

bool collideSphereBox(
    const SphereShape& sphere,
    const math::Vec3& sphere_position,
    const BoxShape& box,
    const math::Vec3& box_position,
    const math::Quat& box_orientation,
    CollisionResult& result)
{
    const math::Mat3 rotation = glm::mat3_cast(box_orientation);
    // Rotations are orthogonal, so the inverse is exactly the transpose and
    // avoids a full Gauss-Jordan elimination per pair.
    const math::Mat3 inverse_rotation = glm::transpose(rotation);

    const math::Vec3 sphere_center_local =
        inverse_rotation * (sphere_position - box_position);
    const math::Vec3 closest_local = clampVector(
        sphere_center_local,
        -box.half_extents,
        box.half_extents);

    const math::Vec3 delta = sphere_center_local - closest_local;
    const double distance_squared = glm::dot(delta, delta);

    if (distance_squared > sphere.radius * sphere.radius)
    {
        return false;
    }

    const double distance = std::sqrt(distance_squared);
    math::Vec3 normal_local{};

    if (distance > 1e-12)
    {
        normal_local = delta / distance;
        result.penetration = sphere.radius - distance;
        result.point = box_position + rotation * closest_local;
    }
    else
    {
        const math::Vec3 face_distances = box.half_extents - glm::abs(sphere_center_local);
        const auto min_it = std::min_element(
            &face_distances[0],
            &face_distances[0] + 3);
        const std::size_t axis = static_cast<std::size_t>(min_it - &face_distances[0]);

        normal_local = math::Vec3(0.0);
        normal_local[axis] = sphere_center_local[axis] >= 0.0 ? 1.0 : -1.0;
        result.penetration = *min_it + sphere.radius;
        result.point = box_position + rotation * closest_local;
    }

    result.collided = true;
    result.normal = rotation * normal_local;
    return true;
}

bool collideBoxPlane(
    const BoxShape& box,
    const math::Vec3& box_position,
    const math::Quat& box_orientation,
    const PlaneShape& plane,
    const math::Vec3& plane_position,
    const math::Quat& plane_orientation,
    CollisionResult& result)
{
    const math::Vec3 normal = planeNormal(plane, plane_orientation);
    const math::Vec3 point = planePoint(plane, plane_position, plane_orientation);
    const math::Vec3 half_extent = worldHalfExtent(box_orientation, box.half_extents);

    const double distance = glm::dot(box_position - point, normal);
    const double extent = glm::dot(half_extent, glm::abs(normal));

    if (std::abs(distance) >= extent)
    {
        return false;
    }

    result.collided = true;
    result.penetration = extent - std::abs(distance);
    result.normal = distance >= 0.0 ? -normal : normal;
    result.point = box_position - normal * distance;
    return true;
}

bool collideBoxBox(
    const BoxShape& box_a,
    const math::Vec3& position_a,
    const math::Quat& orientation_a,
    const BoxShape& box_b,
    const math::Vec3& position_b,
    const math::Quat& orientation_b,
    CollisionResult& result)
{
    const math::Mat3 rotation_a = glm::mat3_cast(orientation_a);
    const math::Mat3 rotation_b = glm::mat3_cast(orientation_b);
    const math::Vec3 half_extent_a = worldHalfExtent(orientation_a, box_a.half_extents);
    const math::Vec3 half_extent_b = worldHalfExtent(orientation_b, box_b.half_extents);
    const math::Vec3 center_delta = position_b - position_a;

    std::vector<math::Vec3> axes;
    axes.reserve(15);

    for (int column = 0; column < 3; ++column)
    {
        axes.push_back(rotation_a[column]);
        axes.push_back(rotation_b[column]);
    }

    for (int column_a = 0; column_a < 3; ++column_a)
    {
        for (int column_b = 0; column_b < 3; ++column_b)
        {
            const math::Vec3 axis = glm::cross(rotation_a[column_a], rotation_b[column_b]);
            if (glm::dot(axis, axis) > 1e-12)
            {
                axes.push_back(glm::normalize(axis));
            }
        }
    }

    double minimum_overlap = std::numeric_limits<double>::infinity();
    math::Vec3 minimum_axis{};

    for (const math::Vec3& raw_axis : axes)
    {
        const math::Vec3 axis = glm::normalize(raw_axis);
        const double radius_a = glm::dot(half_extent_a, glm::abs(axis));
        const double radius_b = glm::dot(half_extent_b, glm::abs(axis));
        const double center_distance = std::abs(glm::dot(center_delta, axis));
        const double overlap = radius_a + radius_b - center_distance;

        if (overlap <= 0.0)
        {
            return false;
        }

        if (overlap < minimum_overlap)
        {
            minimum_overlap = overlap;
            minimum_axis = axis;
        }
    }

    if (glm::dot(center_delta, minimum_axis) < 0.0)
    {
        minimum_axis = -minimum_axis;
    }

    result.collided = true;
    result.penetration = minimum_overlap;
    result.normal = minimum_axis;
    result.point = (position_a + position_b) * 0.5;
    return true;
}

}  // namespace

bool collide(
    const CollisionShape& shape_a,
    const math::Vec3& position_a,
    const math::Quat& orientation_a,
    const CollisionShape& shape_b,
    const math::Vec3& position_b,
    const math::Quat& orientation_b,
    CollisionResult& result)
{
    result = {};

    return std::visit(
        [&](const auto& first) -> bool
        {
            return std::visit(
                [&](const auto& second) -> bool
                {
                    using A = std::decay_t<decltype(first)>;
                    using B = std::decay_t<decltype(second)>;

                    if constexpr (std::is_same_v<A, SphereShape> && std::is_same_v<B, SphereShape>)
                    {
                        return collideSphereSphere(first, position_a, second, position_b, result);
                    }
                    else if constexpr (std::is_same_v<A, SphereShape> && std::is_same_v<B, PlaneShape>)
                    {
                        return collideSpherePlane(first, position_a, second, position_b, orientation_b, result);
                    }
                    else if constexpr (std::is_same_v<A, PlaneShape> && std::is_same_v<B, SphereShape>)
                    {
                        const bool hit = collideSpherePlane(second, position_b, first, position_a, orientation_a, result);
                        if (hit)
                        {
                            result.normal = -result.normal;
                        }
                        return hit;
                    }
                    else if constexpr (std::is_same_v<A, SphereShape> && std::is_same_v<B, BoxShape>)
                    {
                        return collideSphereBox(first, position_a, second, position_b, orientation_b, result);
                    }
                    else if constexpr (std::is_same_v<A, BoxShape> && std::is_same_v<B, SphereShape>)
                    {
                        const bool hit = collideSphereBox(second, position_b, first, position_a, orientation_a, result);
                        if (hit)
                        {
                            result.normal = -result.normal;
                        }
                        return hit;
                    }
                    else if constexpr (std::is_same_v<A, BoxShape> && std::is_same_v<B, PlaneShape>)
                    {
                        return collideBoxPlane(first, position_a, orientation_a, second, position_b, orientation_b, result);
                    }
                    else if constexpr (std::is_same_v<A, PlaneShape> && std::is_same_v<B, BoxShape>)
                    {
                        const bool hit = collideBoxPlane(second, position_b, orientation_b, first, position_a, orientation_a, result);
                        if (hit)
                        {
                            result.normal = -result.normal;
                        }
                        return hit;
                    }
                    else if constexpr (std::is_same_v<A, BoxShape> && std::is_same_v<B, BoxShape>)
                    {
                        return collideBoxBox(first, position_a, orientation_a, second, position_b, orientation_b, result);
                    }
                    else
                    {
                        return false;
                    }
                },
                shape_b);
        },
        shape_a);
}

}  // namespace srp::physics
