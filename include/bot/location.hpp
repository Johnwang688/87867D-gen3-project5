#pragma once

#include "bot/bot.hpp"
#include "bot/types.hpp"

namespace bot {

class Location {
public:
    Location();

    void reset(float x, float y, float heading_deg);
    void start();
    void stop();
    void update();
    Pose get_pose() const;
    bool is_running() const { return _running; }

private:
    static constexpr int NUM_PARTICLES = MCL_NUM_PARTICLES;
    static constexpr float MAX_SENSOR_RANGE  = 2000.0f;
    static constexpr float SENSOR_SIGMA      = 30.0f;
    static constexpr float SENSOR_SIGMA_SQ   = SENSOR_SIGMA * SENSOR_SIGMA;
    static constexpr float TRANS_NOISE_FACTOR = 0.03f;
    static constexpr float TRANS_NOISE_BASE   = 0.5f;
    static constexpr float ROT_NOISE_FACTOR   = 0.02f;
    static constexpr float ROT_NOISE_BASE     = 0.1f;
    static constexpr float ROT_FROM_TRANS     = 0.001f;
    static constexpr float NEFF_RATIO         = 0.5f;
    static constexpr float MIN_SENSOR_READING = 20.0f;
    static constexpr float ENCODER_JUMP_THRESH = 500.0f;
    static constexpr float DEG_TO_RAD = 3.14159265358979f / 180.0f;
    static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979f;
    static constexpr float FIELD_HALF_X = WIDTH  / 2.0f;
    static constexpr float FIELD_HALF_Y = HEIGHT / 2.0f;

    struct SensorConfig {
        float body_x, body_y;   // position in body frame (mm)
        float dir_x,  dir_y;    // unit ray direction in body frame
    };

    static const SensorConfig SENSORS[3];

    Particle _particles[NUM_PARTICLES];
    Particle _resample_buf[NUM_PARTICLES];
    Pose     _pose;

    double _last_left_enc;
    double _last_right_enc;
    double _last_imu_heading;

    volatile bool _running;
    vex::task*    _task;
    uint32_t      _rng_state;

    void predict(float d_center_mm, float d_heading_deg);
    void weight_particles();
    void resample();
    Pose compute_estimate() const;

    float raycast_single(float ox, float oy, float dx, float dy) const;

    inline void body_to_world(float sin_h, float cos_h,
                              float bx, float by,
                              float rx, float ry,
                              float& wx, float& wy) const {
        wx = rx + bx * sin_h + by * cos_h;
        wy = ry - bx * cos_h + by * sin_h;
    }

    float rand_uniform();
    float gaussian_noise(float sigma);

    inline float wrap_heading(float h) const {
        while (h > 180.0f)  h -= 360.0f;
        while (h <= -180.0f) h += 360.0f;
        return h;
    }
};

} // namespace bot
