#pragma once

#include <cstdint>

namespace srp::bridge
{

// Encoder resolution parameters. All values use SI units.
struct EncoderParameters
{
    // Encoder counts per full shaft revolution. Higher values give finer
    // position resolution.
    std::uint32_t counts_per_revolution{100};
};

// Ideal incremental encoder for a rotating shaft.
//
// update() consumes the shaft angle and angular velocity and exposes them
// as:
//
//   - position: shaft angle in radians, or integer encoder counts
//   - speed: shaft angular velocity in rad/s, or counts per second
//
// Counts grow monotonically with the shaft angle using floor rounding, so a
// counterclockwise (negative) spin produces negative counts.
class EncoderModel
{
public:
    explicit EncoderModel(const EncoderParameters& parameters = {});

    void update(double shaft_angle_rad, double shaft_angular_velocity_rad_s);

    double angleRad() const;
    double angularVelocityRadS() const;
    std::int64_t counts() const;
    double countsPerSecond() const;

    void reset();

private:
    EncoderParameters parameters_;
    double angle_rad_{0.0};
    double angular_velocity_rad_s_{0.0};
};

}  // namespace srp::bridge
