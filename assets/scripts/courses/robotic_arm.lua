-- 机械臂课程：抬肩、弯肘、复位循环。

elapsed = 0

function update(dt)
    elapsed = elapsed + dt
    cycle = elapsed % 6.0
    if cycle < 3.0 then
        set_servo(1, 0.8)
        set_servo(2, 0.3)
    else
        set_servo(1, -0.3)
        set_servo(2, 0.8)
    end
    set_servo(3, 0.4)
end
