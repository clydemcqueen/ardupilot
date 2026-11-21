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

#if AP_RANGEFINDER_ENABLED
static AP_AglKalmanFilter::Config agl_kf_config()
{
#if AP_AHRS_NAVEKF3_ENABLED
    return AP_AHRS_NavEKF3::EKF3.get_agl_kf_config();
#else
    return AP_AglKalmanFilter::Config {
        acc_noise      : 0.35f,
        terr_grad_max  : 0.35f,
        rng_noise      : 0.25f,
        rng_innov_gate : 500.0f,
        cos_tilt_min   : 0.71f,
    };
#endif
}
#endif

void Sub::init_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
    rangefinder.init(ROTATION_PITCH_270);
    rangefinder_state.alt_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
    rangefinder_state.agl_kf.InitialiseVariables(MAX(rangefinder.ground_clearance_orient(ROTATION_PITCH_270), 0.05f));
#endif
}

void Sub::update_rangefinder_agl_kf()
{
#if AP_RANGEFINDER_ENABLED
    if (!rangefinder_state.enabled) {
        return;
    }

    Vector3f del_vel_ned;
    float dv_dt;
    if (!ahrs.getCorrectedDeltaVelocityNED(del_vel_ned, dv_dt) || !is_positive(dv_dt)) {
        return;
    }

    rangefinder_state.agl_accum_dv_z += del_vel_ned.z;
    rangefinder_state.agl_accum_dt += dv_dt;

    if (rangefinder_state.agl_accum_dt >= RANGEFINDER_AGL_KF_DT_S) {
        const float dt = rangefinder_state.agl_accum_dt;
        const float vel_dot_ned_z = rangefinder_state.agl_accum_dv_z / dt;
        rangefinder_state.agl_accum_dv_z = 0.0f;
        rangefinder_state.agl_accum_dt = 0.0f;

        float horiz_dist_sq;
        if (ahrs.configured_backend_has_status(AP_AHRS::Status::CONST_POS_MODE) ||
            (!ahrs.configured_backend_has_status(AP_AHRS::Status::HORIZ_POS_REL) &&
             !ahrs.configured_backend_has_status(AP_AHRS::Status::HORIZ_POS_ABS))) {
            horiz_dist_sq = sq(RANGEFINDER_AGL_KF_NOAID_SPD_MS * dt);
        } else {
            const Vector2f &vel_ne = ahrs.groundspeed_vector();
            horiz_dist_sq = sq(vel_ne.x * dt) + sq(vel_ne.y * dt);
        }
        horiz_dist_sq = MIN(horiz_dist_sq, 1.0f);

        const float rng_on_gnd = MAX(rangefinder.ground_clearance_orient(ROTATION_PITCH_270), 0.05f);
        rangefinder_state.agl_kf.predict(AP_HAL::millis(), dt, vel_dot_ned_z, horiz_dist_sq, rng_on_gnd, true, agl_kf_config());
    }

    if (g2.surftrak_filt == 1 && rangefinder_alt_ok()) {
        const float h_agl_now = rangefinder_state.agl_kf.hgt + rangefinder_state.agl_kf.vel * rangefinder_state.agl_accum_dt;
        rangefinder_state.inertial_alt_cm = pos_control.get_pos_estimate_U_m() * 100.0f;
        rangefinder_state.rangefinder_terrain_offset_cm = rangefinder_state.inertial_alt_cm - h_agl_now * 100.0f;
    }
#endif
}

// return rangefinder altitude in centimeters
void Sub::read_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    rangefinder.update();

    // signal quality ranges from 0 (worst) to 100 (perfect), -1 means n/a
    int8_t signal_quality_pct = rangefinder.signal_quality_pct_orient(ROTATION_PITCH_270);

    rangefinder_state.alt_healthy =
            (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) &&
            (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX) &&
            (signal_quality_pct == -1 || signal_quality_pct >= g.rangefinder_signal_min);

    const float raw_dist_m = rangefinder.distance_orient(ROTATION_PITCH_270);
    float temp_alt_m = raw_dist_m;

#if RANGEFINDER_TILT_CORRECTION
    // correct alt for angle of the rangefinder
    temp_alt_m = temp_alt_m * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#endif

    rangefinder_state.alt = temp_alt_m;
    rangefinder_state.inertial_alt_cm = pos_control.get_pos_estimate_U_m() * 100.0f;
    rangefinder_state.min = rangefinder.min_distance_orient(ROTATION_PITCH_270);
    rangefinder_state.max = rangefinder.max_distance_orient(ROTATION_PITCH_270);

    // calculate rangefinder_terrain_offset_cm
    if (rangefinder_state.alt_healthy) {
        uint32_t now = AP_HAL::millis();
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_filt.reset(rangefinder_state.alt);
        } else {
            rangefinder_state.alt_filt.apply(rangefinder_state.alt, 0.05f);
        }
        const uint32_t reading_ms = rangefinder.last_reading_ms(ROTATION_PITCH_270);
        if (reading_ms != rangefinder_state.last_reading_ms) {
            rangefinder_state.last_reading_ms = reading_ms;
            const float rng_on_gnd = MAX(rangefinder.ground_clearance_orient(ROTATION_PITCH_270), 0.05f);
            rangefinder_state.agl_kf.fuse_range(now, raw_dist_m, ahrs.get_rotation_body_to_ned().c.z, rng_on_gnd, agl_kf_config(), true);
        }
        rangefinder_state.last_healthy_ms = now;
        if (g2.surftrak_filt == 1 && rangefinder_state.agl_kf.valid) {
            const float h_agl_now = rangefinder_state.agl_kf.hgt + rangefinder_state.agl_kf.vel * rangefinder_state.agl_accum_dt;
            rangefinder_state.rangefinder_terrain_offset_cm =
                sub.rangefinder_state.inertial_alt_cm - (h_agl_now * 100.0f);
        } else {
            rangefinder_state.rangefinder_terrain_offset_cm =
                sub.rangefinder_state.inertial_alt_cm - (sub.rangefinder_state.alt_filt.get() * 100);
        }
    }

    // send rangefinder altitude and health to waypoint navigation library
    wp_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled,
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
    circle_nav.set_rangefinder_terrain_U_cm(
            rangefinder_state.enabled && wp_nav.rangefinder_used(),
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
#endif  // AP_RANGEFINDER_ENABLED
}

// return true if rangefinder_alt can be used
bool Sub::rangefinder_alt_ok() const
{
#if AP_RANGEFINDER_ENABLED
    if (g2.surftrak_filt == 1 && !rangefinder_state.agl_kf.valid) {
        return false;
    }
#endif
    uint32_t now = AP_HAL::millis();
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy && now - rangefinder_state.last_healthy_ms < RANGEFINDER_TIMEOUT_MS);
}
