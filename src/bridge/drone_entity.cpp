#include "bridge/drone_entity.hpp"

#include "physics/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace srp::bridge
{

struct DroneEntity::Impl
{
    DroneParameters parameters;
    physics::PhysicsWorld world;
    physics::BodyId chassis_id{physics::kInvalidBodyId};
    QuadcopterForceModel quadcopter;
    ImuModel imu;
    DistanceSensorModel distance_sensor;
    double elapsed_time{0.0};
    double throttle{0.0};
    math::Vec3 previous_linear_velocity{0.0};
    bool has_previous_velocity{false};

    explicit Impl(const DroneParameters& values)
        : parameters(values),
          quadcopter(values.quadcopter),
          distance_sensor(DistanceSensorParameters{})
    {
        if (parameters.chassis_mass <= 0.0)
        {
            throw std::invalid_argument("drone chassis mass must be positive");
        }

        if (parameters.max_rotor_angular_velocity_rad_s <= 0.0)
        {
            throw std::invalid_argument(
                "drone max rotor angular velocity must be positive");
        }

        physics::RigidBodyState ground_state;
        ground_state.type = physics::RigidBodyType::kStatic;
        world.createBody(ground_state, physics::PlaneShape{});

        physics::RigidBodyState chassis_state;
        chassis_state.type = physics::RigidBodyType::kDynamic;
        chassis_state.mass = parameters.chassis_mass;
        chassis_state.position = math::Vec3(0.0, 0.25, 0.0);

        constexpr double kHalfWidth = 0.25;
        constexpr double kHalfHeight = 0.05;
        constexpr double kHalfDepth = 0.25;
        const double mass = parameters.chassis_mass;
        math::Mat3 inertia_local(0.0);
        inertia_local[0][0] = mass / 12.0 * (
            std::pow(2.0 * kHalfHeight, 2.0) +
            std::pow(2.0 * kHalfDepth, 2.0));
        inertia_local[1][1] = mass / 12.0 * (
            std::pow(2.0 * kHalfWidth, 2.0) +
            std::pow(2.0 * kHalfDepth, 2.0));
        inertia_local[2][2] = mass / 12.0 * (
            std::pow(2.0 * kHalfWidth, 2.0) +
            std::pow(2.0 * kHalfHeight, 2.0));
        chassis_state.inertia_local = inertia_local;

        physics::BoxShape chassis_shape;
        chassis_shape.half_extents =
            math::Vec3(kHalfWidth, kHalfHeight, kHalfDepth);
        chassis_id = world.createBody(chassis_state, chassis_shape);

        distance_sensor.setIgnoredBody(chassis_id);
        distance_sensor.setPose(chassis_state.position, chassis_state.orientation);
        distance_sensor.update(world);
    }
};

DroneEntity::DroneEntity(const DroneParameters& parameters)
    : impl_(std::make_unique<Impl>(parameters))
{
}

DroneEntity::~DroneEntity() = default;

void DroneEntity::setThrottle(double value)
{
    impl_->throttle = std::clamp(value, 0.0, 1.0);
}

double DroneEntity::throttle() const
{
    return impl_->throttle;
}

void DroneEntity::step(double dt)
{
    if (dt <= 0.0)
    {
        return;
    }

    physics::RigidBodyState* body = impl_->world.body(impl_->chassis_id);
    if (body == nullptr)
    {
        return;
    }

    const double rotor_angular_velocity =
        impl_->throttle * impl_->parameters.max_rotor_angular_velocity_rad_s;
    for (std::size_t i = 0; i < QuadcopterForceModel::kRotorCount; ++i)
    {
        impl_->quadcopter.setRotorAngularVelocity(i, rotor_angular_velocity);
    }

    // The demo keeps the body level: lift is always straight up and the
    // rotor torques cancel at equal speeds.
    body->orientation = math::Quat(1.0, 0.0, 0.0, 0.0);
    body->angular_velocity = math::Vec3(0.0);

    const math::Vec3 body_force = impl_->quadcopter.force();
    const math::Vec3 body_torque = impl_->quadcopter.torque();
    impl_->world.applyForce(impl_->chassis_id, body_force);
    impl_->world.applyTorque(impl_->chassis_id, body_torque);

    impl_->world.step(dt);
    impl_->elapsed_time += dt;

    const math::Vec3 linear_velocity = body->linear_velocity;
    const math::Vec3 acceleration =
        impl_->has_previous_velocity
            ? (linear_velocity - impl_->previous_linear_velocity) / dt
            : math::Vec3(0.0);
    impl_->previous_linear_velocity = linear_velocity;
    impl_->has_previous_velocity = true;

    impl_->imu.update(*body, acceleration);

    impl_->distance_sensor.setPose(body->position, body->orientation);
    impl_->distance_sensor.update(impl_->world);
}

const char* DroneEntity::kind() const
{
    return "drone";
}

double DroneEntity::altitude() const
{
    return impl_->distance_sensor.distance();
}

double DroneEntity::verticalVelocity() const
{
    const physics::RigidBodyState* body = impl_->world.body(impl_->chassis_id);
    return body != nullptr ? body->linear_velocity.y : 0.0;
}

double DroneEntity::elapsedTime() const
{
    return impl_->elapsed_time;
}

const physics::RigidBodyState* DroneEntity::body() const
{
    return impl_->world.body(impl_->chassis_id);
}

const physics::CollisionShape& DroneEntity::bodyShape() const
{
    static const physics::CollisionShape kFallback = physics::BoxShape{};
    const physics::CollisionShape* shape =
        impl_->world.shape(impl_->chassis_id);
    return shape != nullptr ? *shape : kFallback;
}

const ImuModel& DroneEntity::imu() const
{
    return impl_->imu;
}

const DistanceSensorModel& DroneEntity::distanceSensor() const
{
    return impl_->distance_sensor;
}

const QuadcopterForceModel& DroneEntity::quadcopter() const
{
    return impl_->quadcopter;
}

}  // namespace srp::bridge
