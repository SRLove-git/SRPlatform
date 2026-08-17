-- 循迹小车课程（墙沿循迹）。
--
-- 左侧距离传感器 id=11、右侧 id=12 由编辑器注入，返回前方障碍物距离。
-- 目标：让左右读数差保持为 0，即沿障碍物边缘稳定行驶。

target_gap = 0.0
kp = 1.6

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
    left = read_sensor(11)
    right = read_sensor(12)
    if left ~= left then
        left = 4.0
    end
    if right ~= right then
        right = 4.0
    end

    gap = left - right
    set_servo(1, clamp(-gap * kp, -1.0, 1.0))
    set_motor(1, 0.6)
end
