-- 四旋翼悬停课程：PD 高度保持。

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
