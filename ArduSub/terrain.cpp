#include "Sub.h"

// update terrain data
void Sub::terrain_update()
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if RANGEFINDER_ENABLED == ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    }
#endif
#endif
}

// log terrain data - should be called at 1hz
void Sub::terrain_logging()
{
#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif
}

#if SURFTRAK_EXPERIMENTS == ENABLED
Sub::TerrainKF::TerrainKF(float _measurement_nse, float _process_nse) :
        process_nse(_process_nse),
        measurement_nse(_measurement_nse),
        H(1, 0, 0),
        R(_measurement_nse)
{
    reset(0);
}

void Sub::TerrainKF::reset(float z)
{
    x = Vector3f(z, 0, 0);
    P = Matrix3f(measurement_nse, 0, 0, 0, measurement_nse, 0, 0, 0, measurement_nse);
}

void Sub::TerrainKF::predict(float dt)
{
    Matrix3f F(1, dt, 0.5f * dt * dt, 0, 1, dt, 0, 0, 1);
    Matrix3f Q(0, 0, 0, 0, 0, 0, 0, 0, process_nse * dt);

    x = F * x;
    P = F * P * F.transposed() + Q;
}

void Sub::TerrainKF::update(float z)
{
    Vector3f PHT = P * H;
    float S = H * PHT + R;
    Vector3f K = PHT * (1 / S);
    float y = z - H * x;

    x = x + K * y;
    P = P - K.mul_rowcol(H) * P;
}

// Project the current state forward by dt, and return it. The internal state is not changed.
Vector3f Sub::TerrainKF::project(float dt) const
{
    Matrix3f F(1, dt, 0.5f * dt * dt, 0, 1, dt, 0, 0, 1);
    return F * x;
}
#endif  // SURFTRAK_EXPERIMENTS