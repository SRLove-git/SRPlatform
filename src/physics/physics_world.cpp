#include "physics/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/quaternion.hpp>

namespace srp::physics
{

namespace
{

double inverseMass(const RigidBodyState& state)
{
    if (state.type != RigidBodyType::kDynamic || state.mass <= 0.0)
    {
        return 0.0;
    }

    return 1.0 / state.mass;
}

math::Mat3 inverseInertiaWorld(const RigidBodyState& state)
{
    if (state.type != RigidBodyType::kDynamic)
    {
        return math::Mat3(0.0);
    }

    const double determinant = glm::determinant(state.inertia_local);
    if (std::abs(determinant) < 1e-12)
    {
        return math::Mat3(0.0);
    }

    const math::Mat3 rotation = glm::mat3_cast(state.orientation);
    const math::Mat3 inverse_local = glm::inverse(state.inertia_local);
    return rotation * inverse_local * glm::transpose(rotation);
}

math::Vec3 velocityAtPoint(const RigidBodyState& state, const math::Vec3& point)
{
    return state.linear_velocity +
           glm::cross(state.angular_velocity, point - state.position);
}

}  // namespace

PhysicsWorld::PhysicsWorld() = default;

BodyId PhysicsWorld::createBody(const RigidBodyState& state, CollisionShape shape)
{
    const BodyId id = next_body_id_++;
    body_indices_[id] = bodies_.size();
    body_ids_.push_back(id);
    bodies_.push_back(state);
    shapes_.push_back(std::move(shape));
    return id;
}

RigidBodyState* PhysicsWorld::body(BodyId id)
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return nullptr;
    }

    return &bodies_[it->second];
}

const RigidBodyState* PhysicsWorld::body(BodyId id) const
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return nullptr;
    }

    return &bodies_[it->second];
}

CollisionShape* PhysicsWorld::shape(BodyId id)
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return nullptr;
    }

    return &shapes_[it->second];
}

const CollisionShape* PhysicsWorld::shape(BodyId id) const
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return nullptr;
    }

    return &shapes_[it->second];
}

void PhysicsWorld::setGravity(const math::Vec3& gravity)
{
    gravity_ = gravity;
}

math::Vec3 PhysicsWorld::gravity() const
{
    return gravity_;
}

std::vector<Contact> PhysicsWorld::generateContacts() const
{
    std::vector<Contact> contacts;

    for (std::size_t i = 0; i < bodies_.size(); ++i)
    {
        for (std::size_t j = i + 1; j < bodies_.size(); ++j)
        {
            const auto contact = generateContact(
                body_ids_[i],
                shapes_[i],
                bodies_[i].position,
                bodies_[i].orientation,
                body_ids_[j],
                shapes_[j],
                bodies_[j].position,
                bodies_[j].orientation);

            if (contact.has_value())
            {
                contacts.push_back(*contact);
            }
        }
    }

    return contacts;
}

void PhysicsWorld::solveContacts(const std::vector<Contact>& contacts)
{
    constexpr int kVelocityIterations = 10;
    constexpr double kRestitution = 0.0;
    constexpr double kPositionCorrectionPercent = 0.8;
    constexpr double kPenetrationSlop = 0.01;

    for (int iteration = 0; iteration < kVelocityIterations; ++iteration)
    {
        for (const Contact& contact : contacts)
        {
            RigidBodyState* body_a = body(contact.body_a);
            RigidBodyState* body_b = body(contact.body_b);

            if (body_a == nullptr || body_b == nullptr)
            {
                continue;
            }

            const double inverse_mass_a = inverseMass(*body_a);
            const double inverse_mass_b = inverseMass(*body_b);
            const double total_inverse_mass = inverse_mass_a + inverse_mass_b;

            if (total_inverse_mass <= 0.0)
            {
                continue;
            }

            const math::Vec3 normal = contact.point.normal;
            const math::Vec3 relative_velocity =
                velocityAtPoint(*body_b, contact.point.point) -
                velocityAtPoint(*body_a, contact.point.point);
            const double normal_velocity = glm::dot(relative_velocity, normal);

            if (normal_velocity >= 0.0)
            {
                continue;
            }

            const math::Mat3 inverse_inertia_a = inverseInertiaWorld(*body_a);
            const math::Mat3 inverse_inertia_b = inverseInertiaWorld(*body_b);
            const math::Vec3 radius_a = contact.point.point - body_a->position;
            const math::Vec3 radius_b = contact.point.point - body_b->position;
            const math::Vec3 cross_a = glm::cross(radius_a, normal);
            const math::Vec3 cross_b = glm::cross(radius_b, normal);

            const double denominator =
                total_inverse_mass +
                glm::dot(cross_a, inverse_inertia_a * cross_a) +
                glm::dot(cross_b, inverse_inertia_b * cross_b);

            if (denominator <= 0.0)
            {
                continue;
            }

            const double impulse_magnitude =
                -(1.0 + kRestitution) * normal_velocity / denominator;
            const math::Vec3 impulse = normal * impulse_magnitude;

            body_a->linear_velocity -= inverse_mass_a * impulse;
            body_a->angular_velocity -= inverse_inertia_a * glm::cross(radius_a, impulse);
            body_b->linear_velocity += inverse_mass_b * impulse;
            body_b->angular_velocity += inverse_inertia_b * glm::cross(radius_b, impulse);
        }
    }

    for (const Contact& contact : contacts)
    {
        RigidBodyState* body_a = body(contact.body_a);
        RigidBodyState* body_b = body(contact.body_b);

        if (body_a == nullptr || body_b == nullptr)
        {
            continue;
        }

        const double inverse_mass_a = inverseMass(*body_a);
        const double inverse_mass_b = inverseMass(*body_b);
        const double total_inverse_mass = inverse_mass_a + inverse_mass_b;

        if (total_inverse_mass <= 0.0)
        {
            continue;
        }

        const double correction =
            std::max(contact.point.penetration - kPenetrationSlop, 0.0) *
            kPositionCorrectionPercent / total_inverse_mass;
        const math::Vec3 correction_vector = contact.point.normal * correction;

        body_a->position -= inverse_mass_a * correction_vector;
        body_b->position += inverse_mass_b * correction_vector;
    }
}

void PhysicsWorld::step(double dt)
{
    for (RigidBodyState& state : bodies_)
    {
        if (state.type != RigidBodyType::kDynamic)
        {
            continue;
        }

        state.linear_velocity += gravity_ * dt;
        state.position += state.linear_velocity * dt;

        const math::Quat angular_velocity_quaternion(
            0.0,
            state.angular_velocity.x,
            state.angular_velocity.y,
            state.angular_velocity.z);

        state.orientation += 0.5 * dt * (angular_velocity_quaternion * state.orientation);
        normalizeOrientation(state);
    }

    const std::vector<Contact> contacts = generateContacts();
    solveContacts(contacts);
}

}  // namespace srp::physics
