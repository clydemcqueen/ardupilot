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
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
    rangefinder.init(ROTATION_PITCH_270);
    rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#if SURFTRAK_EXPERIMENTS == ENABLED
    terrain_kf = new TerrainKF(g.surftrak_proc_nse, g.surftrak_meas_nse);
#endif
#endif
}

// return rangefinder altitude in centimeters
void Sub::read_rangefinder()
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    // Signal quality ranges from 0 (worst) to 100 (perfect), -1 means n/a
    auto signal_quality_pct = rangefinder.signal_quality_pct_orient(ROTATION_PITCH_270);

    rangefinder_state.alt_healthy =
            (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) &&
            (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX) &&
            (signal_quality_pct == -1 || signal_quality_pct > RANGEFINDER_SIGNAL_CUTOFF);

    int16_t temp_alt = rangefinder.distance_cm_orient(ROTATION_PITCH_270);

#if RANGEFINDER_TILT_CORRECTION == ENABLED
    // Correct for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#endif

    rangefinder_state.alt_cm = temp_alt;
    rangefinder_state.inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    rangefinder_state.min_cm = rangefinder.min_distance_cm_orient(ROTATION_PITCH_270);
    rangefinder_state.max_cm = rangefinder.max_distance_cm_orient(ROTATION_PITCH_270);

    // Calculate and use rangefinder_state.rangefinder_terrain_offset_cm
    if (rangefinder_state.alt_healthy) {
#if SURFTRAK_EXPERIMENTS == ENABLED
        float rangefinder_terrain_offset_cm = sub.rangefinder_state.inertial_alt_cm - (float)sub.rangefinder_state.alt_cm;
#endif

        uint32_t now = AP_HAL::millis();
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // Reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
#if SURFTRAK_EXPERIMENTS == ENABLED
            terrain_kf->reset(rangefinder_terrain_offset_cm);
#endif
        } else {
            // Update filter
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
#if SURFTRAK_EXPERIMENTS == ENABLED
            terrain_kf->predict(0.05f);
            terrain_kf->update(rangefinder_terrain_offset_cm);
#endif
        }
        rangefinder_state.last_healthy_ms = now;

#if SURFTRAK_EXPERIMENTS == ENABLED
        // Project filter forward
        Vector3f projected_state  = terrain_kf->project(sub.g.surftrak_delay);

        // Experiment with several ways to compute rangefinder_terrain_offset_cm
        switch (sub.g.surftrak_calc) {
            case 0:
                // Raw rangefinder reading
                rangefinder_state.rangefinder_terrain_offset_cm = rangefinder_terrain_offset_cm;
                break;
            case 1:
                // Lowpass filter
                rangefinder_state.rangefinder_terrain_offset_cm =
                        sub.rangefinder_state.inertial_alt_cm - sub.rangefinder_state.alt_cm_filt.get();
                break;
            case 2:
                // Kalman filter
                rangefinder_state.rangefinder_terrain_offset_cm = sub.terrain_kf->get_terrain_cm();
                break;
            default:
                // Kalman filter and project forward
                rangefinder_state.rangefinder_terrain_offset_cm = projected_state.x;
                break;
        }

        AP::logger().Write("SURF", "TimeUS,DSAcm,SAcm,Alt,Lp,Kz,Kxp,Kxv,Kxa,Kxpn,Kxvn,PSCot,PSCo", "Qfhffffffffff",
                           AP_HAL::micros64(),

                           // Rangefinder state
                           (double)mode_rnghold.get_rangefinder_target_cm(),        // RF target
                           rangefinder_state.alt_cm,                                // RF reading
                           (double)rangefinder_state.inertial_alt_cm,               // Inertial alt at time of reading

                           // Lowpass filter state
                           (double)rangefinder_state.alt_cm_filt.get(),

                           // Kalman filter state
                           (double)rangefinder_terrain_offset_cm,   // Terrain measurement = Alt - RF
                           (double)terrain_kf->get_x().x,           // Est terrain position
                           (double)terrain_kf->get_x().y,           // Est terrain velocity
                           (double)terrain_kf->get_x().z,           // Est terrain acceleration
                           (double)projected_state.x,               // Est terrain position now
                           (double)projected_state.y,               // Est terrain position now

                           // Position control managed by controllers
                           (double)rangefinder_state.rangefinder_terrain_offset_cm, // PSC offset target
                           (double)pos_control.get_pos_offset_z_cm());              // PSC offset
#else
        rangefinder_state.rangefinder_terrain_offset_cm =
                sub.rangefinder_state.inertial_alt_cm - sub.rangefinder_state.alt_cm_filt.get();
#endif
    }

    // Send rangefinder information to wp_nav and circle_nav controllers
    wp_nav.set_rangefinder_terrain_offset(
            rangefinder_state.enabled,
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
    circle_nav.set_rangefinder_terrain_offset(
            rangefinder_state.enabled && wp_nav.rangefinder_used(),
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
    rangefinder_state.inertial_alt_cm = 0;
    rangefinder_state.rangefinder_terrain_offset_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Sub::rangefinder_alt_ok() const
{
    uint32_t now = AP_HAL::millis();
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy && now - rangefinder_state.last_healthy_ms < RANGEFINDER_TIMEOUT_MS);
}
