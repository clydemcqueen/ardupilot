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

#define INVALID_TARGET (-1)
#define HAS_VALID_TARGET (target_rangefinder_cm > 0)

ModeRnghold::ModeRnghold() :
        target_rangefinder_cm(INVALID_TARGET),
        pilot_in_control(false),
        pilot_control_start_z_cm(0)
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

/*
 * Set the rangefinder target, return true if successful
 */
bool ModeRnghold::set_target_rangefinder_cm(float target_cm)
{
    bool success = false;

    if (sub.control_mode != Number::RNG_HOLD) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "wrong mode, rangefinder target not set");
    } else if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %g meters to set rangefinder target", sub.g.surftrak_depth * 0.01f);
    } else if (target_cm < (float)sub.rangefinder_state.min_cm) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target below minimum, holding depth");
    } else if (target_cm > (float)sub.rangefinder_state.max_cm) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target above maximum, holding depth");
    } else {
        success = true;
    }

    if (success) {
        target_rangefinder_cm = target_cm;
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %g meters", target_rangefinder_cm * 0.01f);

        // Initialize the terrain offset
        auto terrain_offset_cm = sub.inertial_nav.get_position_z_up_cm() - target_rangefinder_cm;
        sub.pos_control.set_pos_offset_z_cm(terrain_offset_cm);
        sub.pos_control.set_pos_offset_target_z_cm(terrain_offset_cm);

    } else {
        reset();
    }

    return success;
}

void ModeRnghold::reset()
{
    target_rangefinder_cm = INVALID_TARGET;

    // Reset the terrain offset
    sub.pos_control.set_pos_offset_z_cm(0);
    sub.pos_control.set_pos_offset_target_z_cm(0);
}

/*
 * Main controller, call at 100hz+
 */
void ModeRnghold::control_range() {
    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // Desired_climb_rate returns 0 when within the deadzone
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (pilot_in_control) {
            // Pilot has released control; apply the delta to the target rangefinder
            set_target_rangefinder_cm(target_rangefinder_cm + inertial_nav.get_position_z_up_cm() - pilot_control_start_z_cm);
            pilot_in_control = false;
        }
        if (sub.ap.at_surface) {
            // Set target depth to 5 cm below SURFACE_DEPTH and reset
            position_control->set_pos_target_z_cm(MIN(position_control->get_pos_target_z_cm(), g.surface_depth - 5.0f));
            reset();
        } else if (sub.ap.at_bottom) {
            // Set target depth to 10 cm above bottom and reset
            position_control->set_pos_target_z_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_target_z_cm()));
            reset();
        } else {
            // Typical operation
            update_surface_offset();
        }
    } else if (HAS_VALID_TARGET && !pilot_in_control) {
        // Pilot has taken control; note the current depth
        pilot_control_start_z_cm = inertial_nav.get_position_z_up_cm();
        pilot_in_control = true;
    }

    // Set the target altitude from the climb rate and the terrain offset
    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);

    // Run the PID controllers
    position_control->update_z_controller();
}

/*
 * Update the AC_PosControl terrain offset if we have a good rangefinder reading
 */
void ModeRnghold::update_surface_offset()
{
    if (sub.rangefinder_alt_ok()) {
        // Get the latest terrain offset
        float rangefinder_terrain_offset_cm = sub.rangefinder_state.rangefinder_terrain_offset_cm;

        // Handle the first reading or a reset
        if (!HAS_VALID_TARGET && sub.rangefinder_state.inertial_alt_cm < sub.g.surftrak_depth) {
            set_target_rangefinder_cm(sub.rangefinder_state.inertial_alt_cm - rangefinder_terrain_offset_cm);
        }

        if (HAS_VALID_TARGET) {
            // Will the new offset target cause the sub to ascend above SURFTRAK_DEPTH?
            float desired_z_cm = rangefinder_terrain_offset_cm + target_rangefinder_cm;
            if (desired_z_cm >= sub.g.surftrak_depth) {
                // Adjust the terrain offset to stay below SURFTRAK_DEPTH, this should avoid "at_surface" events
                rangefinder_terrain_offset_cm += sub.g.surftrak_depth - desired_z_cm;
            }

            // Set the offset target, AC_PosControl will do the rest
            sub.pos_control.set_pos_offset_target_z_cm(rangefinder_terrain_offset_cm);
        }
    }
}
