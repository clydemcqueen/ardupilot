-- Move sub forward at a constant speed

-- TODO add frame_id to API, support frame_id 10 (above terrain)

local guided_mode_num = 4
local dt = 0.05

gcs:send_text(5,"sub_move_forward.lua running")

-- Get position relative to home, in meters NED
-- Is a better way to do this?
function get_position_NED()
    local loc = ahrs:get_location()

    if not loc then
        gcs:send_text(2, "cannot get current location")
        return nil
    end

    pos_NEU_cm = loc.get_vector_from_origin_NEU(loc)
    if not pos_NEU_cm then
        gcs:send_text(2, "cannot get vector from home location")
        return nil
    end

    pos_NED = Vector3f()
    pos_NED:x(pos_NEU_cm:x() * 0.01)
    pos_NED:y(pos_NEU_cm:y() * 0.01)
    pos_NED:z(pos_NEU_cm:z() * -0.01)

    return pos_NED
end

function update()
    if arming:is_armed() and vehicle:get_mode() == guided_mode_num then
        -- Get parameters
        local wpnav_speed = param:get('WPNAV_SPEED') * 0.01

        -- Get current position
        current_pos_NED = get_position_NED()
        if not current_pos_NED then
            return update, dt * 1000
        end

        -- Get current heading, this will be the target heading
        local current_yaw = ahrs:get_yaw()

        -- TODO don't let the depth drift, keep the original z

        -- Target is ~1s ahead at the current heading
        local target_pos_NED = Vector3f()
        target_pos_NED:x(current_pos_NED:x() + wpnav_speed * math.cos(current_yaw))
        target_pos_NED:y(current_pos_NED:y() + wpnav_speed * math.sin(current_yaw))
        target_pos_NED:z(current_pos_NED:z())

        local target_vel_NED = Vector3f()
        target_vel_NED:x(wpnav_speed * math.cos(current_yaw))
        target_vel_NED:y(wpnav_speed * math.sin(current_yaw))
        target_vel_NED:z(0)

        -- Send target
        if not vehicle:set_target_posvel_NED(target_pos_NED, target_vel_NED) then
            gcs:send_text(2, "set_target_posvel_NED failed")
        end
    end

    return update, dt * 1000
end

return update()