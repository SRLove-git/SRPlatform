#include "editor/sim_observer.hpp"

#include "physics/physics_world.hpp"

#include <cmath>
#include <glm/gtc/quaternion.hpp>

namespace srp::editor
{

SimObserver::SimObserver(std::size_t samples_per_channel)
    : capacity_(std::max<std::size_t>(1, samples_per_channel))
{
}

void SimObserver::setSimTime(double time)
{
    sim_time_ = time;
}

double SimObserver::simTime() const
{
    return sim_time_;
}

void SimObserver::record(const std::string& channel, double value)
{
    ensureChannel(channel).record(sim_time_, value);
}

void SimObserver::recordSensor(const std::string& channel, double value)
{
    ensureChannel("sensor_" + channel).record(sim_time_, value);
}

void SimObserver::accumulate(
    const std::string& channel,
    double value_per_second,
    double dt)
{
    const double total = accumulators_[channel] + value_per_second * dt;
    accumulators_[channel] = total;
    record(channel, total);
}

TimeSeries& SimObserver::channel(const std::string& name)
{
    return ensureChannel(name);
}

const TimeSeries& SimObserver::channel(const std::string& name) const
{
    static const TimeSeries empty_series;
    const auto it = channels_.find(name);
    return it == channels_.end() ? empty_series : it->second;
}

bool SimObserver::hasChannel(const std::string& name) const
{
    return channels_.find(name) != channels_.end();
}

const std::vector<std::string>& SimObserver::channelNames() const
{
    return channel_names_;
}

void SimObserver::clear()
{
    channels_.clear();
    channel_names_.clear();
}

TimeSeries& SimObserver::ensureChannel(const std::string& name)
{
    const auto it = channels_.find(name);
    if (it != channels_.end())
    {
        return it->second;
    }

    const auto result = channels_.emplace(
        name,
        TimeSeries(capacity_));
    channel_names_.push_back(name);
    return result.first->second;
}

void sampleCarTelemetry(
    const bridge::CarEntity& car,
    const physics::PhysicsWorld& world,
    double dt,
    SimObserver& observer)
{
    const double time = car.elapsedTime();
    observer.setSimTime(time);

    observer.record("battery_voltage_v", car.batteryVoltage());
    observer.record("motor_current_a", car.motorCurrent());
    observer.record("motor_speed_rad_s", car.motorAngularVelocity());

    const physics::RigidBodyState* chassis = car.chassisBody();
    const double chassis_speed = chassis != nullptr
        ? glm::length(chassis->linear_velocity)
        : 0.0;
    const double chassis_x = chassis != nullptr ? chassis->position.x : 0.0;
    observer.record("chassis_x_m", chassis_x);
    observer.record("chassis_speed_m_s", chassis_speed);

    double kinetic_energy = 0.0;
    double contact_force = 0.0;
    for (const physics::Contact& contact : world.contacts())
    {
        contact_force += contact.point.normal_impulse / std::max(dt, 1e-12);
    }

    for (const physics::BodyId id : world.bodyIds())
    {
        const physics::RigidBodyState* body = world.body(id);
        if (body == nullptr || body->type != physics::RigidBodyType::kDynamic)
        {
            continue;
        }

        kinetic_energy += 0.5 * body->mass *
            glm::dot(body->linear_velocity, body->linear_velocity);
        const math::Vec3 inertia_world_angular =
            glm::mat3_cast(body->orientation) * body->inertia_local *
            glm::transpose(glm::mat3_cast(body->orientation)) *
            body->angular_velocity;
        kinetic_energy += 0.5 *
            glm::dot(body->angular_velocity, inertia_world_angular);
    }

    observer.record("kinetic_energy_j", kinetic_energy);
    observer.record("contact_force_n", contact_force);
    observer.record("contact_count", static_cast<double>(world.contacts().size()));
    observer.accumulate(
        "battery_energy_consumed_j",
        car.batteryVoltage() * car.motorCurrent(),
        dt);
}

}  // namespace srp::editor
