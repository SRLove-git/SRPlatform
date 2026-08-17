#include "bridge/car_entity.hpp"

#include "core/math/types.hpp"
#include "physics/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace srp::bridge
{

struct CarEntity::Impl
{
    CarParameters parameters;
    BatteryModel battery;
    DcMotorModel motor;
    physics::PhysicsWorld world;
    physics::BodyId chassis_id{physics::kInvalidBodyId};
    physics::BodyId wheel_id{physics::kInvalidBodyId};
    physics::JointId wheel_joint_id{physics::kInvalidJointId};
    StateRecorder recorder;
    double elapsed_time{0.0};
    double throttle{0.0};
    double steering{0.0};
    double heading{0.0};

    explicit Impl(const CarParameters& values)
        : parameters(values),
          battery(values.battery),
          motor(values.motor)
    {
        if (parameters.wheel_radius <= 0.0)
        {
            throw std::invalid_argument("car wheel radius must be positive");
        }

        if (parameters.chassis_mass <= 0.0 || parameters.wheel_mass <= 0.0)
        {
            throw std::invalid_argument("car body masses must be positive");
        }

        physics::RigidBodyState ground_state;
        ground_state.type = physics::RigidBodyType::kStatic;
        world.createBody(ground_state, physics::PlaneShape{});

        const double radius = parameters.wheel_radius;
        const double wheel_y = radius * 0.8;
        constexpr double kChassisHalfHeight = 0.1;
        constexpr double kChassisClearance = 0.05;

        physics::RigidBodyState wheel_state;
        wheel_state.type = physics::RigidBodyType::kDynamic;
        wheel_state.mass = parameters.wheel_mass;
        wheel_state.position = math::Vec3(0.0, wheel_y, 0.0);
        const double wheel_inertia = 0.4 * parameters.wheel_mass * radius * radius;
        wheel_state.inertia_local = math::Mat3(wheel_inertia);
        wheel_id = world.createBody(wheel_state, physics::SphereShape{radius});

        physics::RigidBodyState chassis_state;
        chassis_state.type = physics::RigidBodyType::kDynamic;
        chassis_state.mass = parameters.chassis_mass;
        chassis_state.position = math::Vec3(
            0.0,
            wheel_y + radius + kChassisHalfHeight + kChassisClearance,
            0.0);
        chassis_state.inertia_local = math::Mat3(0.1);
        physics::BoxShape chassis_shape;
        chassis_shape.half_extents = math::Vec3(0.2, kChassisHalfHeight, 0.2);
        chassis_id = world.createBody(chassis_state, chassis_shape);

        physics::JointDefinition wheel_joint;
        wheel_joint.type = physics::JointType::kWheel;
        wheel_joint.body_a = chassis_id;
        wheel_joint.body_b = wheel_id;
        wheel_joint.anchor_local_a = math::Vec3(
            0.0,
            -(kChassisHalfHeight + kChassisClearance),
            0.0);
        wheel_joint.anchor_local_b = math::Vec3(0.0);
        wheel_joint.axis_local_a = math::Vec3(0.0, 0.0, 1.0);
        wheel_joint.axis_local_b = math::Vec3(0.0, 0.0, 1.0);
        wheel_joint.wheel_radius = radius;
        wheel_joint.drive_torque = 0.0;
        wheel_joint_id = world.createJoint(wheel_joint);

        if (wheel_joint_id == physics::kInvalidJointId)
        {
            throw std::runtime_error("failed to create car wheel joint");
        }
    }
};

CarEntity::CarEntity(const CarParameters& parameters)
    : impl_(std::make_unique<Impl>(parameters))
{
}

CarEntity::~CarEntity() = default;

void CarEntity::setThrottle(double value)
{
    impl_->throttle = std::clamp(value, -1.0, 1.0);
}

double CarEntity::throttle() const
{
    return impl_->throttle;
}

void CarEntity::setSteering(double value)
{
    impl_->steering = std::clamp(value, -1.0, 1.0);
}

double CarEntity::steering() const
{
    return impl_->steering;
}

double CarEntity::heading() const
{
    return impl_->heading;
}

void CarEntity::step(double dt)
{
    if (dt <= 0.0)
    {
        return;
    }

    const double battery_voltage = batteryVoltage();
    const double supply_voltage = impl_->throttle * battery_voltage;

    const physics::RigidBodyState* wheel_state = wheelBody();
    if (wheel_state == nullptr)
    {
        return;
    }

    const double wheel_angular_velocity = wheel_state->angular_velocity.z;
    const double direction = (wheel_angular_velocity > 0.0) -
                             (wheel_angular_velocity < 0.0);
    const double load_torque =
        direction * impl_->parameters.rolling_resistance_torque_nm +
        impl_->parameters.viscous_load_nm_per_rad_s * wheel_angular_velocity;

    impl_->motor.setAngularVelocity(wheel_angular_velocity);
    impl_->motor.step(supply_voltage, load_torque, dt);
    impl_->battery.step(std::abs(impl_->motor.current()), dt);

    physics::Joint* wheel_joint = impl_->world.joint(impl_->wheel_joint_id);
    if (wheel_joint != nullptr)
    {
        wheel_joint->drive_torque = impl_->motor.electricalTorque();
    }

    impl_->world.step(dt);
    impl_->elapsed_time += dt;

    const physics::RigidBodyState* chassis_state = chassisBody();
    if (chassis_state != nullptr)
    {
        constexpr double kMaxSteeringRate = 1.0;
        const double speed_direction =
            (impl_->throttle > 0.0) - (impl_->throttle < 0.0);
        impl_->heading += impl_->steering * kMaxSteeringRate * speed_direction * dt;
    }

    CarStateSample sample;
    sample.time_s = impl_->elapsed_time;
    sample.battery_voltage_v = batteryVoltage();
    sample.motor_current_a = motorCurrent();
    sample.motor_angular_velocity_rad_s = motorAngularVelocity();
    sample.chassis_position_x_m =
        chassis_state != nullptr ? chassis_state->position.x : 0.0;
    impl_->recorder.record(sample);
}

const char* CarEntity::kind() const
{
    return "car";
}

double CarEntity::batteryStateOfCharge() const
{
    return impl_->battery.stateOfCharge();
}

double CarEntity::batteryVoltage() const
{
    return impl_->battery.terminalVoltage(impl_->motor.current());
}

double CarEntity::motorCurrent() const
{
    return impl_->motor.current();
}

double CarEntity::motorAngularVelocity() const
{
    return impl_->motor.angularVelocity();
}

double CarEntity::elapsedTime() const
{
    return impl_->elapsed_time;
}

const physics::RigidBodyState* CarEntity::chassisBody() const
{
    return impl_->world.body(impl_->chassis_id);
}

const physics::RigidBodyState* CarEntity::wheelBody() const
{
    return impl_->world.body(impl_->wheel_id);
}

const physics::CollisionShape& CarEntity::chassisShape() const
{
    static const physics::CollisionShape kFallback = physics::BoxShape{};
    const physics::CollisionShape* shape = impl_->world.shape(impl_->chassis_id);
    return shape != nullptr ? *shape : kFallback;
}

const physics::CollisionShape& CarEntity::wheelShape() const
{
    static const physics::CollisionShape kFallback = physics::SphereShape{};
    const physics::CollisionShape* shape = impl_->world.shape(impl_->wheel_id);
    return shape != nullptr ? *shape : kFallback;
}

const StateRecorder& CarEntity::recorder() const
{
    return impl_->recorder;
}

}  // namespace srp::bridge
