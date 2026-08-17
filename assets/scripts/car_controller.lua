-- Example RC car controller. Runs at every fixed simulation step.
elapsed = 0

function update(dt)
    elapsed = elapsed + dt
    if elapsed < 2.0 then
        set_motor(1, 1.0)
        set_servo(1, 0.0)
    elseif elapsed < 4.0 then
        set_motor(1, 0.0)
        set_servo(1, 0.6)
    elseif elapsed < 6.0 then
        set_motor(1, 0.8)
        set_servo(1, -0.6)
    elseif elapsed < 8.0 then
        set_motor(1, -0.5)
        set_servo(1, 0.0)
    else
        elapsed = 0.0
    end
end
