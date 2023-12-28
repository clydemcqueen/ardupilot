// Sketch: estimate the ground position using a Kalman filter

// Build and run:
// ./waf configure --board sitl
// ./waf build --target examples/terrain_kf
// ./build/sitl/examples/terrain_kf -M vectored -C

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <chrono>
#include <thread>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

struct TerrainKF {
    TerrainKF(float _dt, float measurement_var, float process_var) :
            t(0),
            dt(_dt),
            x(0, 0, 0),
            P(50, 0, 0, 0, 50, 0, 0, 0, 0.1),
            Q(0, 0, 0, 0, 0, 0, 0, 0, sqrtf(process_var) * _dt),
            F(1, _dt, 0.5f * _dt * _dt, 0, 1, _dt, 0, 0, 1),
            H(1, 0, 0),
            R(measurement_var) {}

    void predict()
    {
        t += dt;

        // x_ = F_ * x_;
        x = F * x;

        // P_ = F_ * P_ * F_.transpose() + Q_;
        P = F * P * F.transposed() + Q;
    }

    void update(float z)
    {
        // Eigen::MatrixXd PHT = P_ * H_.transpose();
        Vector3f PHT = P * H;

        // S_ = H_ * PHT + R;
        float S = H * PHT + R;

        // K_ = PHT * S_.inverse();
        Vector3f K = PHT * (1 / S);

        // y_ = z - H_ * x_;
        float y = z - H * x;

        // x_ = x_ + K_ * y_;
        x = x + K * y;

        // P_ = P_ - K_ * H_ * P_;
        P = P - K.mul_rowcol(H) * P;
    }

    float t;
    float dt;
    Vector3f x;
    Matrix3f P;
    Matrix3f Q;
    Matrix3f F;
    Vector3f H;
    float R;
};

void gen_flat(float dt, float val, std::vector<float> &result);
void gen_flat(float dt, float val, std::vector<float> &result)
{
    const int steps = (int)(10 / dt);
    for (int i = 0; i < steps; ++i) {
        result.push_back(val);
    }
}

void gen_ramp(float dt, float start, float stop, float rate, std::vector<float> &result);
void gen_ramp(float dt, float start, float stop, float rate, std::vector<float> &result)
{
    const float step = rate * dt;
    const int steps = (int)((stop - start) / step);
    for (int i = 0; i < steps; ++i) {
        result.push_back(start + (float)i * step);
    }
}

void gen_trap(float dt, std::vector<float> &result);
void gen_trap(float dt, std::vector<float> &result)
{
    gen_flat(dt, -20, result);
    gen_ramp(dt, -20, -16, 0.5, result);
    gen_flat(dt, -16, result);
    gen_ramp(dt, -16, -20, -0.5, result);
}

TerrainKF *terrain_kf = nullptr;
std::vector<float> terrain;
long unsigned int ix = 0;

void setup()
{
    terrain_kf = new TerrainKF(0.05, 0.01, 0.01);

    // Generate some terrain
    gen_trap(0.05, terrain);

    hal.console->printf("timestamp, pos, vel, acc, gt\n");
}

void loop()
{
    terrain_kf->predict();
    terrain_kf->update(terrain[ix]);

    hal.console->printf("%g, %g, %g, %g, %g\n", terrain_kf->t, terrain_kf->x.x, terrain_kf->x.y, terrain_kf->x.z, terrain[ix]);

    // Wrap terrain
    if (++ix >= terrain.size()) {
        ix = 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(uint16_t(terrain_kf->dt * 1000.0)));
}

AP_HAL_MAIN()
