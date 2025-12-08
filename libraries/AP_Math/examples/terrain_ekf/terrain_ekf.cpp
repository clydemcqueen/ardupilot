// A Terrain EKF with 3 states: terrain_z, slope_n, slope_e

// This is an ArduPilot sketch.
// To build and run:
// ./waf configure --board sitl
// ./waf build --target examples/terrain_ekf
// ./build/sitl/examples/terrain_ekf -M vectored -C

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <chrono>
#include <thread>
#include <vector>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// TerrainEKF class ported from Python
class TerrainEKF {
public:
    TerrainEKF(float initial_terrain_z,
               float initial_slope_n,
               float initial_slope_e,
               float initial_terrain_variance,
               float initial_slope_variance,
               float terrain_process_noise,
               float slope_process_noise,
               float _gate_threshold) :
        gate_threshold(_gate_threshold)
    {
        // Initial state
        x.x = initial_terrain_z;
        x.y = initial_slope_n;
        x.z = initial_slope_e;

        // Initial covariance
        P.zero();
        P[0][0] = initial_terrain_variance;
        P[1][1] = initial_slope_variance;
        P[2][2] = initial_slope_variance;

        // Process noise
        Q_per_sec.zero();
        Q_per_sec[0][0] = terrain_process_noise;
        Q_per_sec[1][1] = slope_process_noise;
        Q_per_sec[2][2] = slope_process_noise;
    }

    void set_beam_vectors(const Vector3f beams[4]) {
        for (uint8_t i = 0; i < 4; i++) {
            beam_vectors_body[i] = beams[i];
        }
    }

    // Predict step
    void predict(float vn, float ve, float dt) {
        // F = [[1, -vn*dt, -ve*dt], [0, 1, 0], [0, 0, 1]]
        Matrix3f F;
        F.identity();
        F[0][1] = -vn * dt;
        F[0][2] = -ve * dt;

        // Q = Q_per_sec * dt
        Matrix3f Q = Q_per_sec * dt;

        // x = F @ x
        x = F * x;

        // P = F @ P @ F.T + Q
        P = F * P * F.transposed() + Q;
    }

    // Update step
    void update(const float beams[4], float rov_depth, const Matrix3f &R_body_to_earth, float beam_variance, bool verbose) {
        float est_terrain_z = x.x;
        float est_slope_n = x.y;
        float est_slope_e = x.z;
        float est_alt = est_terrain_z - rov_depth;

        if (est_alt < 0.1f) {
            if (verbose) {
                hal.console->printf("est_alt %.2f < 0.1, skipping update\n", est_alt);
            }
            return;
        }

        Vector3f n_earth(est_slope_n, est_slope_e, 1.0f);

        for (uint8_t i = 0; i < 4; i++) {
            float measured_range = beams[i];

            if (measured_range <= 0.0f) {
                if (verbose) hal.console->printf("Reject beam %d: bad value %.2f\n", i, measured_range);
                continue;
            }

            Vector3f v_body = beam_vectors_body[i];
            Vector3f v_earth = R_body_to_earth * v_body;
            float dot_prod = v_earth * n_earth; 

            if (dot_prod <= 0.01f) {
                 // Try to recover or just skip
                if (verbose) hal.console->printf("Reject beam %d: nearly parallel %.2f\n", i, dot_prod);
                continue;
            }

            float expected_range = est_alt / dot_prod;
            float residual = measured_range - expected_range;

            // Jacobian H
            float dR_dT = 1.0f / dot_prod;
            float dR_dSn = -(est_alt * v_earth.x) / (dot_prod * dot_prod);
            float dR_dSe = -(est_alt * v_earth.y) / (dot_prod * dot_prod);
            
            // H is 1x3 row vector
            Vector3f H_row(dR_dT, dR_dSn, dR_dSe);

            // S = H P H^T + R
            // P * H^T (3x1 column)
            Vector3f PHt = P * H_row; 
            
            float S = (H_row * PHt) + beam_variance; 
            float nis = (residual * residual) / S;

            if (nis > gate_threshold) {
                if (verbose) {
                     hal.console->printf("Reject beam %d: NIS %.1f > %.1f (res=%.2f)\n", i, nis, gate_threshold, residual);
                }
                continue;
            }

            // Kalman Gain K = P H^T * (1/S)
            Vector3f K = PHt * (1.0f / S);

            // Update State
            x = x + K * residual;

            // Update Covariance (Joseph Form)
            // P = (I - KH) P (I - KH)^T + K R K^T
            Matrix3f I;
            I.identity();
            
            // KH Outer Product
            Matrix3f KH;
            for(int r=0; r<3; r++) {
                for(int c=0; c<3; c++) {
                    KH[r][c] = K[r] * H_row[c];
                }
            }

            Matrix3f I_KH = I - KH;
            
            // Term 1: (I-KH) * P * (I-KH)^T
            Matrix3f Term1 = I_KH * P * I_KH.transposed();
            
            // Term 2: K * R * K^T
            // K is 3x1. K*K^T is 3x3 outer product.
            Matrix3f KRKt;
            for(int r=0; r<3; r++) {
                for(int c=0; c<3; c++) {
                    KRKt[r][c] = K[r] * K[c] * beam_variance;
                }
            }
            
            P = Term1 + KRKt;

            // Ensure symmetry
            P = (P + P.transposed()) * 0.5f;
            
            // Re-calc for next iter?
            est_terrain_z = x.x;
            est_slope_n = x.y;
            est_slope_e = x.z;
            est_alt = est_terrain_z - rov_depth;
            n_earth = Vector3f(est_slope_n, est_slope_e, 1.0f);
        }
    }

    Vector3f get_state() const { return x; }

private:
    Vector3f x; // [terrain_z, slope_n, slope_e]
    Matrix3f P;
    Matrix3f Q_per_sec;
    float gate_threshold;
    Vector3f beam_vectors_body[4];
};

TerrainEKF *ekf = nullptr;

void setup()
{
    hal.console->printf("TerrainEKF Test\n");

    // Initialize EKF
    // initial_terrain_z = 20.0 (guess)
    // slopes = 0
    ekf = new TerrainEKF(20.0f, 0.0f, 0.0f, 100.0f, 1.0f, 0.01f, 0.01f, 9.0f); // lowered process noise for slope

    // Setup Beam Vectors (Example: X config, 30 deg down)
    // 30 deg from vertical.
    float angle_from_vertical = radians(30.0f);
    float cz = cosf(angle_from_vertical);
    float sz = sinf(angle_from_vertical);
    
    Vector3f beams[4];
    // Beam 0: 45 deg azimuth (Front-Right)
    beams[0] = Vector3f(sz*cosf(radians(45.0f)), sz*sinf(radians(45.0f)), cz);
    // Beam 1: 135 deg azimuth (Rear-Right)
    beams[1] = Vector3f(sz*cosf(radians(135.0f)), sz*sinf(radians(135.0f)), cz);
    // Beam 2: 225 deg azimuth (Rear-Left)
    beams[2] = Vector3f(sz*cosf(radians(225.0f)), sz*sinf(radians(225.0f)), cz);
    // Beam 3: 315 deg azimuth (Front-Left)
    beams[3] = Vector3f(sz*cosf(radians(315.0f)), sz*sinf(radians(315.0f)), cz);

    ekf->set_beam_vectors(beams);

    hal.console->printf("Initialized. True Terrain Z assumed 15.0m.\n");
}

void loop()
{
    static float t = 0.0f;
    float dt = 0.02f; // 50Hz
    t += dt;

    // Simulation
    float true_terrain_z = 15.0f; // Flat terrain
    float rov_depth = 5.0f; // ROV at 5m depth
    float true_alt = true_terrain_z - rov_depth; // 10m altitude

    // Prediction with some velocity
    float vn = 1.0f; // 1 m/s North
    float ve = 0.0f; 
    ekf->predict(vn, ve, dt);

    // Update with measurements
    // Synthesize beam ranges
    // For flat terrain, range = altitude / cos(angle_from_vertical)
    float angle_from_vertical = radians(30.0f);
    float range_true = true_alt / cosf(angle_from_vertical);
    
    // Add noise?
    float noise = 0.0f; // Perfect meas
    
    float measurements[4];
    for(int i=0; i<4; i++) measurements[i] = range_true + noise;

    // Identity rotation (level vehicle)
    Matrix3f R;
    R.identity();

    ekf->update(measurements, rov_depth, R, 0.01f, true); // 0.01 variance

    Vector3f state = ekf->get_state();
    hal.console->printf("t=%.2f Z_est=%.2f Sn=%.3f Se=%.3f (Truth: 15.0)\n", 
                        t, state.x, state.y, state.z);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (t > 5.0f) {
        exit(0);
    }
}

AP_HAL_MAIN()
