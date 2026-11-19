/*
  2-state IMU-aided AGL Kalman Filter

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_Math/AP_Math.h>

/*
 * 2-state IMU-aided AGL Kalman Filter
 * Uses bias-corrected NED vertical acceleration for prediction and downward
 * rangefinder as measurement, decoupled from the main filter's vertical position state.
 * State: x = [hgt (m, +up), vel (m/s, +up)]
 */
class AP_AglKalmanFilter {
public:
    struct Config {
        ftype acc_noise;        // accelerometer process noise (m/s^2)
        ftype terr_grad_max;    // RMS terrain gradient (dimensionless)
        ftype rng_noise;        // rangefinder measurement noise (m)
        ftype rng_innov_gate;   // innovation gate (% of 1-sigma, e.g. 500 -> 5.0)
        ftype cos_tilt_min;     // minimum cos(tilt) required to fuse rangefinder
    };

    void InitialiseVariables(ftype initialHgt);

    // Run IMU state and covariance prediction step
    void predict(uint32_t now_ms,
                 ftype imuDt,
                 ftype velDotNED_z,
                 ftype horizDistSq,
                 ftype rngOnGnd,
                 bool waitingForRng,
                 const Config &cfg);

    // Fuse a downward rangefinder reading (uncorrected slant range in meters)
    void fuse_range(uint32_t now_ms,
                    ftype rng,
                    ftype cosTilt,
                    ftype rngOnGnd,
                    const Config &cfg,
                    bool resetIfInvalid = false);

    // Combined step matching NavEKF3_core::UpdateAglKf()
    void update(uint32_t now_ms,
                ftype imuDt,
                ftype velDotNED_z,
                ftype horizDistSq,
                ftype rngOnGnd,
                bool rangeDataToFuse,
                ftype rng,
                ftype cosTilt,
                const Config &cfg,
                bool resetIfInvalid = false);

    ftype hgt;                      // AGL height estimate (m, positive up from ground)
    ftype vel;                      // AGL velocity estimate (m/s, positive = climbing)
    ftype cov[2][2];                // 2x2 covariance matrix (upper triangle, symmetric)
    bool  valid;                    // true when RF has been fused within the last 5 s
    uint32_t lastRngFuseTime_ms;    // timestamp of last successful RF fusion into AGL KF
};
