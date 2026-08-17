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

double mass(const RigidBodyState& state)
{
    if (state.type != RigidBodyType::kDynamic || state.mass <= 0.0)
    {
        return 0.0;
    }

    return state.mass;
}

double inertiaAboutAxis(const RigidBodyState& state, const math::Vec3& local_axis)
{
    if (state.type != RigidBodyType::kDynamic)
    {
        return 0.0;
    }

    const double inertia = glm::dot(local_axis, state.inertia_local * local_axis);
    return std::max(0.0, inertia);
}

const Contact* findContact(const std::vector<Contact>& contacts, BodyId body_id)
{
    for (const Contact& contact : contacts)
    {
        if (contact.body_a == body_id || contact.body_b == body_id)
        {
            return &contact;
        }
    }

    return nullptr;
}

math::Vec3 normalPointingToBody(const Contact& contact, BodyId body_id)
{
    if (contact.body_b == body_id)
    {
        return contact.point.normal;
    }

    return -contact.point.normal;
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
    force_accumulators_.emplace_back(0.0);
    torque_accumulators_.emplace_back(0.0);
    return id;
}

JointId PhysicsWorld::createJoint(const JointDefinition& definition)
{
    if (body_indices_.find(definition.body_a) == body_indices_.end() ||
        body_indices_.find(definition.body_b) == body_indices_.end())
    {
        return kInvalidJointId;
    }

    const JointId id = next_joint_id_++;
    joint_indices_[id] = joints_.size();
    joints_.push_back(definition);
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

const std::vector<BodyId>& PhysicsWorld::bodyIds() const
{
    return body_ids_;
}

Joint* PhysicsWorld::joint(JointId id)
{
    const auto it = joint_indices_.find(id);
    if (it == joint_indices_.end())
    {
        return nullptr;
    }

    return &joints_[it->second];
}

const Joint* PhysicsWorld::joint(JointId id) const
{
    const auto it = joint_indices_.find(id);
    if (it == joint_indices_.end())
    {
        return nullptr;
    }

    return &joints_[it->second];
}

const std::vector<Contact>& PhysicsWorld::contacts() const
{
    return last_contacts_;
}

void PhysicsWorld::setGravity(const math::Vec3& gravity)
{
    gravity_ = gravity;
}

math::Vec3 PhysicsWorld::gravity() const
{
    return gravity_;
}

void PhysicsWorld::applyForce(BodyId id, const math::Vec3& force)
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return;
    }

    force_accumulators_[it->second] += force;
}

void PhysicsWorld::applyTorque(BodyId id, const math::Vec3& torque)
{
    const auto it = body_indices_.find(id);
    if (it == body_indices_.end())
    {
        return;
    }

    torque_accumulators_[it->second] += torque;
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
            const double restitution = std::max(body_a->restitution, body_b->restitution);
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
                -(1.0 + restitution) * normal_velocity / denominator;
            const math::Vec3 impulse = normal * impulse_magnitude;

            body_a->linear_velocity -= inverse_mass_a * impulse;
            body_a->angular_velocity -= inverse_inertia_a * glm::cross(radius_a, impulse);
            body_b->linear_velocity += inverse_mass_b * impulse;
            body_b->angular_velocity += inverse_inertia_b * glm::cross(radius_b, impulse);

            math::Vec3 tangent = relative_velocity - normal * normal_velocity;
            const double tangent_speed_squared = glm::dot(tangent, tangent);
            if (tangent_speed_squared > 1e-12)
            {
                tangent /= std::sqrt(tangent_speed_squared);

                const math::Vec3 cross_a_tangent = glm::cross(radius_a, tangent);
                const math::Vec3 cross_b_tangent = glm::cross(radius_b, tangent);
                const double tangent_denominator =
                    total_inverse_mass +
                    glm::dot(cross_a_tangent, inverse_inertia_a * cross_a_tangent) +
                    glm::dot(cross_b_tangent, inverse_inertia_b * cross_b_tangent);

                if (tangent_denominator > 0.0)
                {
                    const double tangent_velocity = glm::dot(relative_velocity, tangent);
                    const double friction_coefficient =
                        std::sqrt(std::max(0.0, body_a->friction * body_b->friction));
                    const double max_friction_impulse =
                        friction_coefficient * impulse_magnitude;

                    const double friction_impulse_magnitude = std::clamp(
                        -tangent_velocity / tangent_denominator,
                        -max_friction_impulse,
                        max_friction_impulse);

                    const math::Vec3 friction_impulse =
                        tangent * friction_impulse_magnitude;

                    body_a->linear_velocity -= inverse_mass_a * friction_impulse;
                    body_a->angular_velocity -=
                        inverse_inertia_a * glm::cross(radius_a, friction_impulse);
                    body_b->linear_velocity += inverse_mass_b * friction_impulse;
                    body_b->angular_velocity +=
                        inverse_inertia_b * glm::cross(radius_b, friction_impulse);
                }
            }
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

void PhysicsWorld::solveWheelJoint(const Joint& joint, double dt)
{
    if (dt <= 0.0)
    {
        return;
    }

    RigidBodyState* chassis = body(joint.body_a);
    RigidBodyState* wheel = body(joint.body_b);
    if (chassis == nullptr || wheel == nullptr)
    {
        return;
    }

    const double chassis_mass = mass(*chassis);
    const double wheel_mass = mass(*wheel);
    if (chassis_mass <= 0.0 || wheel_mass <= 0.0)
    {
        return;
    }

    const Contact* contact = findContact(last_contacts_, joint.body_b);
    if (contact == nullptr)
    {
        return;
    }

    const math::Vec3 normal = normalPointingToBody(*contact, joint.body_b);
    const math::Vec3 axis = glm::normalize(wheel->orientation * joint.axis_local_b);
    math::Vec3 forward = -glm::cross(axis, normal);

    const double forward_length = glm::length(forward);
    if (forward_length < 1e-12)
    {
        return;
    }
    forward /= forward_length;

    const double radius = joint.wheel_radius;
    if (radius <= 0.0)
    {
        return;
    }

    const math::Vec3 axis_local = glm::normalize(joint.axis_local_b);
    const double inertia_axis = inertiaAboutAxis(*wheel, axis_local);
    if (inertia_axis <= 0.0)
    {
        return;
    }

    const double effective_mass =
        chassis_mass + wheel_mass + inertia_axis / (radius * radius);
    if (effective_mass <= 0.0)
    {
        return;
    }

    const double chassis_speed = glm::dot(chassis->linear_velocity, forward);
    const double wheel_speed = glm::dot(wheel->linear_velocity, forward);
    const double angular_speed = glm::dot(wheel->angular_velocity, axis);

    const double generalized_momentum =
        chassis_mass * chassis_speed +
        wheel_mass * wheel_speed +
        inertia_axis / radius * angular_speed;

    double rolling_speed = generalized_momentum / effective_mass;
    rolling_speed +=
        (joint.drive_torque / radius) / effective_mass * dt;

    chassis->linear_velocity -= forward * chassis_speed;
    wheel->linear_velocity -= forward * wheel_speed;
    wheel->angular_velocity -= axis * angular_speed;

    chassis->linear_velocity += forward * rolling_speed;
    wheel->linear_velocity += forward * rolling_speed;
    wheel->angular_velocity += axis * (rolling_speed / radius);
}

void PhysicsWorld::solveJoints(double dt)
{
    constexpr double kPositionCorrection = 1.0;
    constexpr double kVelocityCorrection = 1.0;

    for (const Joint& joint : joints_)
    {
        if (joint.type == JointType::kWheel)
        {
            solveWheelJoint(joint, dt);
            continue;
        }

        RigidBodyState* body_a = body(joint.body_a);
        RigidBodyState* body_b = body(joint.body_b);

        if (body_a == nullptr || body_b == nullptr)
        {
            continue;
        }

        const math::Vec3 anchor_a =
            body_a->position + body_a->orientation * joint.anchor_local_a;
        const math::Vec3 anchor_b =
            body_b->position + body_b->orientation * joint.anchor_local_b;

        const double inverse_mass_a = inverseMass(*body_a);
        const double inverse_mass_b = inverseMass(*body_b);
        const double total_inverse_mass = inverse_mass_a + inverse_mass_b;

        if (total_inverse_mass <= 0.0)
        {
            continue;
        }

        const math::Vec3 position_error = anchor_b - anchor_a;
        const math::Vec3 position_correction =
            position_error * (kPositionCorrection / total_inverse_mass);

        body_a->position += inverse_mass_a * position_correction;
        body_b->position -= inverse_mass_b * position_correction;

        const math::Vec3 relative_velocity =
            velocityAtPoint(*body_b, anchor_b) -
            velocityAtPoint(*body_a, anchor_a);
        const math::Vec3 velocity_correction =
            relative_velocity * (kVelocityCorrection / total_inverse_mass);

        body_a->linear_velocity += inverse_mass_a * velocity_correction;
        body_b->linear_velocity -= inverse_mass_b * velocity_correction;
    }
}

void PhysicsWorld::step(double dt)
{
    for (std::size_t i = 0; i < bodies_.size(); ++i)
    {
        RigidBodyState& state = bodies_[i];
        if (state.type != RigidBodyType::kDynamic)
        {
            continue;
        }

        state.linear_velocity += gravity_ * dt;
        if (state.mass > 0.0)
        {
            state.linear_velocity +=
                force_accumulators_[i] / state.mass * dt;
            state.angular_velocity +=
                inverseInertiaWorld(state) * torque_accumulators_[i] * dt;
        }
        state.position += state.linear_velocity * dt;

        const math::Quat angular_velocity_quaternion(
            0.0,
            state.angular_velocity.x,
            state.angular_velocity.y,
            state.angular_velocity.z);

        state.orientation += 0.5 * dt * (angular_velocity_quaternion * state.orientation);
        normalizeOrientation(state);
    }

    last_contacts_ = generateContacts();
    solveContacts(last_contacts_);
    solveJoints(dt);

    std::fill(force_accumulators_.begin(), force_accumulators_.end(), math::Vec3(0.0));
    std::fill(torque_accumulators_.begin(), torque_accumulators_.end(), math::Vec3(0.0));
}

}  // namespace srp::physics
