#include "physics/aabb.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/gtc/matrix_transform.hpp>

namespace srp::physics
{

Aabb computeAabb(
    const CollisionShape& shape,
    const math::Vec3& position,
    const math::Quat& orientation)
{
    return std::visit(
        [&](const auto& value) -> Aabb
        {
            using T = std::decay_t<decltype(value)>;

            const math::Mat3 rotation = glm::mat3_cast(orientation);

            if constexpr (std::is_same_v<T, BoxShape>)
            {
                math::Vec3 half_extent{};
                for (int row = 0; row < 3; ++row)
                {
                    half_extent[row] =
                        std::abs(rotation[row][0]) * value.half_extents.x +
                        std::abs(rotation[row][1]) * value.half_extents.y +
                        std::abs(rotation[row][2]) * value.half_extents.z;
                }

                return {position - half_extent, position + half_extent, true};
            }
            else if constexpr (std::is_same_v<T, SphereShape>)
            {
                const math::Vec3 half_extent{value.radius};
                return {position - half_extent, position + half_extent, true};
            }
            else if constexpr (std::is_same_v<T, PlaneShape>)
            {
                return {math::Vec3(0.0), math::Vec3(0.0), false};
            }
            else if constexpr (std::is_same_v<T, CylinderShape>)
            {
                const math::Vec3 axis = rotation * math::Vec3(0.0, 1.0, 0.0);
                math::Vec3 half_extent{};

                for (int i = 0; i < 3; ++i)
                {
                    const double axis_component = std::abs(axis[i]);
                    const double radial_component =
                        std::sqrt(std::max(0.0, 1.0 - axis_component * axis_component));
                    half_extent[i] =
                        axis_component * value.half_height +
                        radial_component * value.radius;
                }

                return {position - half_extent, position + half_extent, true};
            }
            else
            {
                if (value.points.empty())
                {
                    return {math::Vec3(0.0), math::Vec3(0.0), false};
                }

                const double infinity = std::numeric_limits<double>::infinity();
                math::Vec3 min_point(infinity);
                math::Vec3 max_point(-infinity);

                for (const math::Vec3& point : value.points)
                {
                    const math::Vec3 world_point = position + rotation * point;
                    min_point = glm::min(min_point, world_point);
                    max_point = glm::max(max_point, world_point);
                }

                return {min_point, max_point, true};
            }
        },
        shape);
}

bool overlaps(const Aabb& lhs, const Aabb& rhs)
{
    if (!lhs.is_finite || !rhs.is_finite)
    {
        return true;
    }

    return lhs.min.x <= rhs.max.x && lhs.max.x >= rhs.min.x &&
           lhs.min.y <= rhs.max.y && lhs.max.y >= rhs.min.y &&
           lhs.min.z <= rhs.max.z && lhs.max.z >= rhs.min.z;
}

}  // namespace srp::physics
