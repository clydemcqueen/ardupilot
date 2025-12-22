#include "Sub.h"

// return barometric altitude in centimeters
void Sub::read_barometer()
{
    barometer.update();
    // If we are reading a positive altitude, the sensor needs calibration
    // Even a few meters above the water we should have no significant depth reading
    if(barometer.get_altitude() > 0) {
        barometer.update_calibration();
    }

    if (ap.depth_sensor_present) {
        sensor_health.depth = barometer.healthy(depth_sensor_idx);
    }
}

void Sub::init_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
    rangefinder.init(ROTATION_PITCH_270);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Sub::read_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.update();

    rangefinder_state.update();

    if (rangefinder_state.alt_healthy) {
        // Update terrain estimate
        rangefinder_state.terrain_u_m = rangefinder_state.ref_pos_u_m - rangefinder_state.alt_m_filt.get();
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled,
            rangefinder_state.alt_healthy,
            rangefinder_state.terrain_u_m * 100.0f);
    circle_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled && wp_nav.rangefinder_used(),
            rangefinder_state.alt_healthy,
            rangefinder_state.terrain_u_m * 100.0f);
#endif  // AP_RANGEFINDER_ENABLED
}

// return true if rangefinder_alt can be used
bool Sub::rangefinder_alt_ok() const
{
    return rangefinder_state.enabled_and_healthy() && !rangefinder_state.data_stale();
}
