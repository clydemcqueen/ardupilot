#include "Sub.h"

/*
 * RNG_HOLD (rangefinder hold) -- a variation on ALT_HOLD (depth hold)
 *
 * The real work for ALT_HOLD and RNG_HOLD is handled by AC_PosControl, which provides 2 inputs for depth:
 * -- target depth (sub.pos_control._pos_target.z). This is the desired depth, plus an offset.
 * -- target offset (sub.pos_control._pos_offset_target_z). This is the desired offset.
 *
 * ALT_HOLD and RNG_HOLD set the target depth in these situations:
 * -- During initialization, we call pos_control.init_z_controller(). This sets target depth to the current depth.
 * -- If the sub hits the surface or bottom we call pos_control.set_pos_target_z_cm().
 * -- If the pilot takes control we call pos_control.set_pos_target_z_from_climb_rate_cm().
 *
 * At the end of the control loop ALT_HOLD and RNG_HOLD call pos_control.update_z_controller() to pass the buck.
 *
 * ALT_HOLD does not use the target offset.
 *
 * RNG_HOLD sets the target offset to implement surface tracking. This is handled by Sub::SurfaceTracking. We call
 * SurfaceTracking in these situations:
 * -- During initialization, we call surface_tracking.enable().
 * -- During normal operation, we call surface_tracking.update_surface_offset().
 * -- If the sub hits the surface or bottom, or the pilot takes control, we call surface_tracking.reset().
 */

ModeRnghold::ModeRnghold() :
        pilot_in_control(),
        pilot_control_start_z_cm()
{ }

bool ModeRnghold::init(bool ignore_checks)
{
    if (!ModeAlthold::init(ignore_checks)) {
        return false;
    }

    reset();

    if (!sub.rangefinder_alt_ok()) {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "waiting for a rangefinder reading");
    }
    if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %g meters to hold range", sub.g.surftrak_depth * 0.01f);
    }

    return true;
}

void ModeRnghold::run()
{
    run_pre();
    control_range();
    run_post();
}

void ModeRnghold::reset()
{
    target_rangefinder_cm = -1;
    sub.pos_control.set_pos_offset_z_cm(0);
    sub.pos_control.set_pos_offset_target_z_cm(0);
}

void ModeRnghold::control_range() {
    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // desired_climb_rate returns 0 when within the deadzone
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (pilot_in_control) {
            // pilot has released control, apply the delta to the target rangefinder
            apply_delta_cm_or_reset(inertial_nav.get_position_z_up_cm() - pilot_control_start_z_cm);
            pilot_in_control = false;
        }
        if (sub.ap.at_surface) {
            // set target depth to 5 cm below SURFACE_DEPTH
            position_control->set_pos_target_z_cm(MIN(position_control->get_pos_target_z_cm(), g.surface_depth - 5.0f));
            reset();
        } else if (sub.ap.at_bottom) {
            // set target depth to 10 cm above bottom
            position_control->set_pos_target_z_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_target_z_cm()));
            reset();
        } else {
            // normal operation
            update_surface_offset();
        }
    } else if (has_target_rangefinder() && !pilot_in_control) {
        // pilot has taken control, note the current depth
        pilot_control_start_z_cm = inertial_nav.get_position_z_up_cm();
        pilot_in_control = true;
    }

    // set the target z from the climb rate and the z offset, and adjust the z vel and accel targets
    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);

    // run the z vel and accel PID controllers
    position_control->update_z_controller();
}

void ModeRnghold::set_target_rangefinder_cm(float new_target_cm)
{
    if (!sub.rangefinder_alt_ok()) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder not ok, rangefinder target not set");
    } else if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %g meters to set rangefinder target", sub.g.surftrak_depth * 0.01f);
    } else {
        target_rangefinder_cm = new_target_cm;
        auto terrain_offset_cm = sub.inertial_nav.get_position_z_up_cm() - target_rangefinder_cm;
        sub.pos_control.set_pos_offset_z_cm(terrain_offset_cm);
        sub.pos_control.set_pos_offset_target_z_cm(terrain_offset_cm);
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %g m", target_rangefinder_cm * 0.01f);
    }
}

void ModeRnghold::apply_delta_cm_or_reset(float delta_cm)
{
    auto new_target_cm = target_rangefinder_cm + delta_cm;
    if (new_target_cm < (float)sub.rangefinder_state.min_cm) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target below minimum, holding depth");
        reset();
    } else if (new_target_cm > (float)sub.rangefinder_state.max_cm) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target above maximum, holding depth");
        reset();
    } else {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "delta applied %g m", delta_cm * 0.01f);
        set_target_rangefinder_cm(new_target_cm);
    }
}

void ModeRnghold::update_surface_offset()
{
    if (sub.rangefinder_alt_ok()) {
        float rangefinder_terrain_offset_cm = sub.rangefinder_state.rangefinder_terrain_offset_cm;

        // Handle first reading or controller reset
        if (!has_target_rangefinder() && sub.rangefinder_state.inertial_alt_cm < sub.g.surftrak_depth) {
            set_target_rangefinder_cm(sub.rangefinder_state.inertial_alt_cm - rangefinder_terrain_offset_cm);
        }

        if (has_target_rangefinder()) {
            // Will the new offset target cause the sub to ascend above SURFTRAK_DEPTH?
            float desired_z_cm = rangefinder_terrain_offset_cm + target_rangefinder_cm;
            if (desired_z_cm >= sub.g.surftrak_depth) {
                // adjust the offset target to stay below SURFTRAK_DEPTH
                rangefinder_terrain_offset_cm += sub.g.surftrak_depth - desired_z_cm;
            }

            // Set the offset target, AC_PosControl will do the rest
            sub.pos_control.set_pos_offset_target_z_cm(rangefinder_terrain_offset_cm);
        } else {
            sub.pos_control.set_pos_offset_z_cm(0);
            sub.pos_control.set_pos_offset_target_z_cm(0);
        }
    }
}
