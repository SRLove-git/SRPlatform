-- RC Car Demo entry script (Phase 5.8 example).
-- Drives forward for two seconds, then stops.

elapsed = 0

function update(dt)
    elapsed = elapsed + dt
    if elapsed < 2.0 then
        set_motor(1, 1.0)
        set_servo(1, 0.0)
    else
        set_motor(1, 0.0)
    end
end
