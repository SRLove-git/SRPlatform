#include "physics/collision_shape.hpp"

namespace srp::physics
{

CollisionShapeType shapeType(const CollisionShape& shape)
{
    return std::visit(
        [](const auto& value) -> CollisionShapeType
        {
            using T = std::decay_t<decltype(value)>;

            if constexpr (std::is_same_v<T, BoxShape>)
            {
                return CollisionShapeType::kBox;
            }
            else if constexpr (std::is_same_v<T, SphereShape>)
            {
                return CollisionShapeType::kSphere;
            }
            else if constexpr (std::is_same_v<T, PlaneShape>)
            {
                return CollisionShapeType::kPlane;
            }
            else if constexpr (std::is_same_v<T, CylinderShape>)
            {
                return CollisionShapeType::kCylinder;
            }
            else
            {
                return CollisionShapeType::kConvexHull;
            }
        },
        shape);
}

}  // namespace srp::physics
