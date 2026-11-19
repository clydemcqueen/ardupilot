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

#include "AP_AglKalmanFilter.h"

namespace {
constexpr uint32_t aglKfRngTimeout_ms = 5000;   // mark filter invalid / hard-reset after this gap without RF fusion
}

// 2-state AGL KF initialisation
// Start with generous uncertainty; the first valid RF measurement will hard-reset the state
void AP_AglKalmanFilter::InitialiseVariables(ftype initialHgt)
{
    hgt = initialHgt;    // assume sitting on ground at minimum range
    vel = 0.0f;
    cov[0][0] = 25.0f;   // 5 m initial std-dev in height
    cov[0][1] = 0.0f;
    cov[1][0] = 0.0f;
    cov[1][1] = 1.0f;    // 1 m/s initial std-dev in velocity
    valid = false;
    lastRngFuseTime_ms = 0;
}

/*
 * 2-state IMU-aided AGL Kalman filter
 *
 * State:       x  = [h_agl (m), v_agl (m/s)]'   (AGL is "up")
 * Transition:  F  = [[1, imuDt], [0, 1]]
 * Input:       u  = -velDotNED.z * imuDt             (gravity-included accel)
 * Noise:       Qvel = sq(acc_noise * imuDt)
 *              Qhgt = sq(terr_grad_max) * horizDistSq
 *
 * Prediction:  x(k+1) = F*x(k) + [0, u]'
 *              P(k+1) = F*P*F' + Q
 *
 * Observation: z = rng * cosTilt,  H = [1, 0],  R = sq(rng_noise)
 * Update:      innov = z - H*x,  innovVar = H*P*H' + R
 *              K = P*H' / innovVar
 *              x += K * innov
 *              P  = (I-KH)*P*(I-KH)' + K*R*K'
 */
void AP_AglKalmanFilter::predict(uint32_t now_ms,
                                 ftype imuDt,
                                 ftype velDotNED_z,
                                 ftype horizDistSq,
                                 ftype rngOnGnd,
                                 bool waitingForRng,
                                 const Config &cfg)
{
    // h_agl(k+1) = h_agl(k) + v_agl(k)*imuDt
    hgt += vel * imuDt;

    // v_agl(k+1) = v_agl(k) - velDotNED.z*imuDt
    // velDotNED.z is NED-down acceleration (positive = downward, includes gravity).
    // Negate: downward acceleration reduces AGL rate.
    vel -= velDotNED_z * imuDt;

    // First-order decay of v_agl toward zero when RF is absent (tau = 2 s).
    // Without range measurements v_agl is unobservable; accumulated IMU bias
    // error will cause it to drift, pulling h_agl to the floor during
    // subsequent climbs.  The decay limits that drift.
    // At the aglKfRngTimeout_ms validity timeout (5 s), |v| is at most
    // exp(-5/2) ~ 8% of its value at last RF fusion, so the hard reset finds v near zero.
    if (waitingForRng) {
        const ftype tauV = 2.0f;
        vel *= expf(-imuDt / tauV);
    }

    // AGL cannot go below the on-ground sensor reading
    hgt = MAX(hgt, rngOnGnd);

    // ----- Covariance prediction: P = F*P*F' + Q -----
    //
    // F = [[1, imuDt],   state-transition matrix
    //      [0,  1]]
    //
    // Process noise Q:
    //
    // Qhgt — terrain-induced AGL uncertainty during forward flight.
    //   As the vehicle moves horizontally by dist over terrain with unknown gradient "g",
    //   the true AGL changes by ~g * dist.  Since "g" is unknown, we treat it as zero-mean
    //   with std-dev terrGradMax, giving variance: Qhgt = terrGradMax² * horizDist²
    //   horizDist² = (vx²+vy²)*imuDt² is the squared horizontal distance travelled this step.
    //
    // Qvel — unmodelled vertical accelerations (IMU noise, vibration, model error).
    //   Uses the same accNoise parameter as CovariancePrediction (sq(imuDt*accNoise)).
    //
    const ftype Qvel = sq(cfg.acc_noise * imuDt);
    const ftype Qhgt = sq(cfg.terr_grad_max) * horizDistSq;

    // Capture before overwrite (P is symmetric, so P[0][1] == P[1][0])
    const ftype P00 = cov[0][0];
    const ftype P01 = cov[0][1];   // == P[1][0]
    const ftype P11 = cov[1][1];

    // Expanded F*P*F' + Q:
    cov[0][0] = P00 + imuDt * (P01 + P01) + sq(imuDt) * P11 + Qhgt;
    cov[0][1] = cov[1][0] = P01 + imuDt * P11;
    cov[1][1] = P11 + Qvel;

    // Cap covariance to prevent runaway during prolonged RF absence
    cov[0][0] = MIN(cov[0][0], 100.0f);  // 10 m std-dev cap
    cov[1][1] = MIN(cov[1][1], 100.0f);  // 10 m/s std-dev cap

    // mark invalid if RF has been absent too long
    if (waitingForRng && (now_ms - lastRngFuseTime_ms > aglKfRngTimeout_ms)) {
        valid = false;
    }
}

void AP_AglKalmanFilter::fuse_range(uint32_t now_ms,
                                    ftype rng,
                                    ftype cosTilt,
                                    ftype rngOnGnd,
                                    const Config &cfg,
                                    bool resetIfInvalid)
{
    // Only fuse when vehicle tilt is within acceptable limits
    if (cosTilt < cfg.cos_tilt_min) {
        return;
    }

    // After the timeout of IMU-only propagation, vertical velocity drift makes the
    // prediction unreliable.  Re-initialise directly from the rangefinder.
    if ((resetIfInvalid && !valid) || (now_ms - lastRngFuseTime_ms > aglKfRngTimeout_ms)) {
        // Tilt-corrected AGL directly from rangefinder reading
        hgt = MAX(rng * cosTilt, rngOnGnd);
        vel = 0.0f;                          // assume stationary on reset
        cov[0][0] = sq(cfg.rng_noise);       // initialise h uncertainty to RF noise
        cov[0][1] = cov[1][0] = 0.0f;
        cov[1][1] = 1.0f;                    // 1 m/s velocity uncertainty after reset
        lastRngFuseTime_ms = now_ms;
        valid = true;
        return;  // skip measurement update this cycle (just used the reading for reset)
    }

    // Measurement update — fuse tilt-corrected rangefinder reading
    //
    // Observation model: z = h_agl,  H = [1, 0]
    //   z_meas = rng * cos(tilt)
    //
    const ftype hgtMeas = MAX(rng * cosTilt, rngOnGnd);

    // Measurement noise variance R
    const ftype measNoiseVar = sq(cfg.rng_noise);

    // Innovation and innovation covariance
    // hgtInnov = hgtMeas - H*x = hgtMeas - h_agl
    // innovVar = H*P*H' + R    = P[0][0] + R
    const ftype hgtInnov = hgtMeas - hgt;
    const ftype innovVar = cov[0][0] + measNoiseVar;

    // reject outliers (RF glitches, specular reflections, etc.)
    // gate is expressed as a multiplier on the 1-sigma bound.
    const ftype innovGate = MAX(0.01f * cfg.rng_innov_gate, 1.0f);
    if (sq(hgtInnov) > sq(innovGate) * innovVar) {
        // Innovation too large, likely a glitch.  Inflate both height and velocity
        // uncertainty so the next valid reading can correct both states more aggressively.
        cov[0][0] = MIN(cov[0][0] * 2.0f, 100.0f);
        cov[1][1] = MIN(cov[1][1] * 2.0f, 100.0f);
        return;
    }

    // Kalman gain:  K = P*H' / innovVar = [P[0][0]/innovVar, P[1][0]/innovVar]'
    // (H = [1, 0], so P*H' = first column of P)
    const ftype Kh = cov[0][0] / innovVar;
    const ftype Kv = cov[1][0] / innovVar;

    // State update:  x += K * hgtInnov
    hgt += Kh * hgtInnov;
    vel += Kv * hgtInnov;
    hgt  = MAX(hgt, rngOnGnd);  // enforce physical constraint after update

    // Covariance update P = (I-KH)*P*(I-KH)' + K*R*K'
    // With H = [1, 0], (I-KH) = [[1-Kh, 0], [-Kv, 1]]:
    //   P[0][0] = (1-Kh)²*Phh + Kh²*R
    //   P[0][1] = (1-Kh)*(Phv - Kv*Phh) + Kh*Kv*R
    //   P[1][1] = Pvv - 2*Kv*Phv + Kv²*innovVar
    const ftype oneMinusKh = 1.0f - Kh;
    const ftype Phh = cov[0][0];
    const ftype Phv = cov[0][1];
    const ftype Pvv = cov[1][1];

    cov[0][0] = MAX(sq(oneMinusKh) * Phh + sq(Kh) * measNoiseVar, 0.0f);
    cov[0][1] = cov[1][0] = oneMinusKh * (Phv - Kv * Phh) + Kh * Kv * measNoiseVar;
    cov[1][1] = MAX(Pvv - 2.0f * Kv * Phv + sq(Kv) * innovVar, 0.0f);

    lastRngFuseTime_ms = now_ms;
    valid = true;
}

void AP_AglKalmanFilter::update(uint32_t now_ms,
                                ftype imuDt,
                                ftype velDotNED_z,
                                ftype horizDistSq,
                                ftype rngOnGnd,
                                bool rangeDataToFuse,
                                ftype rng,
                                ftype cosTilt,
                                const Config &cfg,
                                bool resetIfInvalid)
{
    predict(now_ms, imuDt, velDotNED_z, horizDistSq, rngOnGnd, !rangeDataToFuse, cfg);

    if (rangeDataToFuse) {
        fuse_range(now_ms, rng, cosTilt, rngOnGnd, cfg, resetIfInvalid);
    }
}
