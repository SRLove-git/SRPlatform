-- Simple altitude-hold hover script (Phase 4.7 example).
--
-- Reads the downward distance sensor (1) and vertical velocity (2), then
-- sets the collective throttle with a PD controller plus hover feedforward.
-- Tune the constants below to change the hover behavior.

target_altitude = 1.0
hover_throttle = 0.58
kp = 0.5
kd = 0.3

function clamp(value, low, high)
    if value < low then
        return low
    end
    if value > high then
        return high
    end
    return value
end

function update(dt)
    altitude = read_sensor(1)
    vertical_velocity = read_sensor(2)
    throttle = hover_throttle + kp * (target_altitude - altitude) - kd * vertical_velocity
    set_motor(1, clamp(throttle, 0.0, 1.0))
end
