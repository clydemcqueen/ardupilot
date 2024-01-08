#include "Sub.h"

#if 0
#if RANGEFINDER_ENABLED == ENABLED

/*
 * Sub::SurfaceTracking -- use a rangefinder sensor to track the seafloor
 *
 * The real work is handled by AC_PosControl.
 *
 * Surface tracking starts in the "reset" state (sub.surface_tracking.reset_target = true). In this state we are waiting
 * for a good rangefinder reading.
 *
 * When we get a good rangefinder reading, we exit the reset state:
 * -- We set the target rangefinder to the current rangefinder.
 * -- We clear the target offset: sub.pos_control.set_pos_offset_target_z_cm(0).
 * -- We clear the offset: sub.pos_control.set_pos_offset_z_cm(0) to clear the offset.
 *
 * During normal operation, SurfaceTracking is using rangefinder readings to set the target offset, and AC_PosControl
 * does the rest.
 *
 * We generally do not want to reset SurfaceTracking if the rangefinder glitches, since that will result in a new
 * target rangefinder. E.g., if a pilot is tracking at a target of 1m above the seafloor, there is a glitch, then the
 * next rangefinder reading shows 1.1m, the desired behavior is to move 10cm closer to the seafloor, vs setting a new
 * target of 1.1m above the seafloor.
 *
 * If the pilot takes control and moves to a new depth there are 2 ways to update the rangefinder target:
 * 1. Call reset(). The next call to update_surface_offset() will set a new rangefinder target. This works well if the
 *    rangefinder readings the depth readings track each other. If the rangefinder readings lag the depth readings, this
 *    can lead to the sub "bouncing back" to a previous location.
 * 2. The caller can track the change in depth, and call apply_delta_or_reset(delta) when the pilot releases control.
 *    The rangefinder target will be adjusted by the delta if possible.
 */

void Sub::SurfaceTracking::enable(bool _enabled)
{
    enabled = _enabled;
    reset();

    if (enabled) {
        if (!sub.rangefinder_alt_ok()) {
            sub.gcs().send_text(MAV_SEVERITY_INFO, "waiting for a rangefinder reading");
        }
        if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
            sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %g meters to hold range", sub.g.surftrak_depth * 0.01f);
        }
    }
}

void Sub::SurfaceTracking::reset()
{
    target_rangefinder_cm = -1;
    sub.pos_control.set_pos_offset_z_cm(0);
    sub.pos_control.set_pos_offset_target_z_cm(0);
}

void Sub::SurfaceTracking::set_target_rangefinder_cm(float new_target_cm)
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

void Sub::SurfaceTracking::apply_delta_cm_or_reset(float delta_cm)
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

void Sub::SurfaceTracking::update_surface_offset()
{
    if (enabled) {
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
    } else {
        sub.pos_control.set_pos_offset_z_cm(0);
        sub.pos_control.set_pos_offset_target_z_cm(0);
    }
}

#endif
#endif