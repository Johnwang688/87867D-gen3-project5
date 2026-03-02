#include "bot/location.hpp"
#include <cstring>
#include <cmath>


namespace {
    bot::Location* s_instance = nullptr;
}

static int location_task_fn() {
    while (s_instance && s_instance->is_running()) {
        uint32_t start = bot::Brain.Timer.time(vex::msec);
        s_instance->update();
        uint32_t elapsed = bot::Brain.Timer.time(vex::msec) - start;
        if (elapsed < 20)
            vex::task::sleep(20 - elapsed);
    }
    return 0;
}


namespace bot {

const float Location::FIELD_HALF_X = WIDTH  / 2.0f;
const float Location::FIELD_HALF_Y = HEIGHT / 2.0f;

const Location::SensorConfig Location::SENSORS[3] = {
    { LEFT_SENSOR_BODY_X,  LEFT_SENSOR_BODY_Y,  -1.0f,  0.0f },
    { RIGHT_SENSOR_BODY_X, RIGHT_SENSOR_BODY_Y,  1.0f,  0.0f },
    { BACK_SENSOR_BODY_X,  BACK_SENSOR_BODY_Y,   0.0f, -1.0f },
};


Location::Location()
    : _pose{0.0, 0.0, 0.0},
      _last_left_enc(0.0), _last_right_enc(0.0), _last_imu_heading(0.0),
      _running(false), _full_update_cycle(true), _task(nullptr),
      _rng_state(0xDEADBEEF), _w_slow(0.0f), _w_fast(0.0f)
{
    float w = 1.0f / NUM_PARTICLES;
    for (int i = 0; i < NUM_PARTICLES; i++)
        _particles[i] = {0.0f, 0.0f, 0.0f, w};
}

void Location::reset(float x, float y, float heading_deg) {
    _state_mutex.lock();

    _rng_state = static_cast<uint32_t>(bot::Brain.Timer.time(vex::msec));
    if (_rng_state == 0) _rng_state = 0xDEADBEEF;

    bot::sensors::imu.set_heading_math(static_cast<double>(heading_deg) + 90.0);

    float w = 1.0f / NUM_PARTICLES;
    for (int i = 0; i < NUM_PARTICLES; i++) {
        _particles[i].x       = x + gaussian_noise(10.0f);
        _particles[i].y       = y + gaussian_noise(10.0f);
        _particles[i].heading = wrap_heading(heading_deg + gaussian_noise(2.0f));
        _particles[i].weight  = w;
    }

    _pose_mutex.lock();
    _pose.x       = static_cast<double>(x);
    _pose.y       = static_cast<double>(y);
    _pose.heading  = static_cast<double>(heading_deg);
    _pose_mutex.unlock();

    _w_slow = 0.0f;
    _w_fast = 0.0f;

    _last_left_enc    = bot::motors::left_dt.position(vex::degrees);
    _last_right_enc   = bot::motors::right_dt.position(vex::degrees);
    _last_imu_heading = bot::sensors::imu.heading(vex::degrees);
    _full_update_cycle = true;

    _state_mutex.unlock();
}

void Location::start() {
    if (_running) return;
    s_instance = this;
    _running   = true;
    _full_update_cycle = true;

    _last_left_enc    = bot::motors::left_dt.position(vex::degrees);
    _last_right_enc   = bot::motors::right_dt.position(vex::degrees);
    _last_imu_heading = bot::sensors::imu.heading(vex::degrees);

    _task = new vex::task(location_task_fn);
}

void Location::stop() {
    _running = false;
    vex::task::sleep(50);
    if (_task) {
        delete _task;
        _task = nullptr;
    }
    s_instance = nullptr;
}

Pose Location::get_pose() const {
    _pose_mutex.lock();
    Pose p = _pose;
    _pose_mutex.unlock();
    return p;
}


void Location::update() {
    _state_mutex.lock();

    double left_enc   = bot::motors::left_dt.position(vex::degrees);
    double right_enc  = bot::motors::right_dt.position(vex::degrees);
    double imu_heading = bot::sensors::imu.heading(vex::degrees);

    float d_left  = static_cast<float>(left_enc  - _last_left_enc);
    float d_right = static_cast<float>(right_enc - _last_right_enc);

    if (fabsf(d_left) > ENCODER_JUMP_THRESH ||
        fabsf(d_right) > ENCODER_JUMP_THRESH) {
        _last_left_enc    = left_enc;
        _last_right_enc   = right_enc;
        _last_imu_heading = imu_heading;
        _state_mutex.unlock();
        return;
    }

    float d_left_mm   = helpers::encoderDegreesToMM(d_left); // fixed encoder tick to mm distance conversion bug
    float d_right_mm  = helpers::encoderDegreesToMM(d_right);
    float d_center_mm = (d_left_mm + d_right_mm) * 0.5f;

    float d_imu = static_cast<float>(imu_heading - _last_imu_heading);
    if (d_imu >  180.0f) d_imu -= 360.0f;
    if (d_imu < -180.0f) d_imu += 360.0f;
    float d_heading_imu = -d_imu;

    float d_heading_odom = (d_right_mm - d_left_mm)
                           / static_cast<float>(TRACK_WIDTH) * RAD_TO_DEG;

    float d_heading = IMU_BLEND * d_heading_imu
                    + (1.0f - IMU_BLEND) * d_heading_odom;

    if (_full_update_cycle)
        resample();

    predict(d_center_mm, d_heading);

    if (_full_update_cycle)
        weight_particles();

    Pose new_pose = compute_estimate();
    _pose_mutex.lock();
    _pose = new_pose;
    _pose_mutex.unlock();

    _last_left_enc    = left_enc;
    _last_right_enc   = right_enc;
    _last_imu_heading = imu_heading;
    _full_update_cycle = !_full_update_cycle;

    _state_mutex.unlock();
}


void Location::predict(float d_center_mm, float d_heading_deg) {
    if (fabsf(d_center_mm) < 0.01f && fabsf(d_heading_deg) < 0.01f) return;

    float abs_d = fabsf(d_center_mm);
    float abs_h = fabsf(d_heading_deg);

    float trans_sigma = TRANS_NOISE_FACTOR * abs_d + TRANS_NOISE_BASE;
    float lat_sigma   = trans_sigma * 0.5f;
    float rot_sigma   = ROT_NOISE_FACTOR * abs_h
                      + ROT_FROM_TRANS * abs_d
                      + ROT_NOISE_BASE;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        float nt = gaussian_noise(trans_sigma);
        float nl = gaussian_noise(lat_sigma);
        float nr = gaussian_noise(rot_sigma);

        float half_dh = (d_heading_deg + nr) * 0.5f;
        float avg_rad = (_particles[i].heading + half_dh) * DEG_TO_RAD;
        float cos_h   = cosf(avg_rad);
        float sin_h   = sinf(avg_rad);
        float d       = d_center_mm + nt;

        _particles[i].x += -d * sin_h - nl * cos_h;
        _particles[i].y +=  d * cos_h - nl * sin_h;
        _particles[i].heading = wrap_heading(
            _particles[i].heading + d_heading_deg + nr);

        if (_particles[i].x < -FIELD_HALF_X + 1.0f)
            _particles[i].x = -FIELD_HALF_X + 1.0f;
        if (_particles[i].x >  FIELD_HALF_X - 1.0f)
            _particles[i].x =  FIELD_HALF_X - 1.0f;
        if (_particles[i].y < -FIELD_HALF_Y + 1.0f)
            _particles[i].y = -FIELD_HALF_Y + 1.0f;
        if (_particles[i].y >  FIELD_HALF_Y - 1.0f)
            _particles[i].y =  FIELD_HALF_Y - 1.0f;
    }
}


void Location::weight_particles() {
    float readings[3];
    bool  valid[3];

    valid[0] = bot::sensors::left_dist.isObjectDetected();
    valid[1] = bot::sensors::right_dist.isObjectDetected();
    valid[2] = bot::sensors::back_dist.isObjectDetected();

    if (valid[0])
        readings[0] = static_cast<float>(
            bot::sensors::left_dist.objectDistance(vex::mm));
    if (valid[1])
        readings[1] = static_cast<float>(
            bot::sensors::right_dist.objectDistance(vex::mm));
    if (valid[2])
        readings[2] = static_cast<float>(
            bot::sensors::back_dist.objectDistance(vex::mm));

    for (int s = 0; s < 3; s++) {
        if (valid[s] &&
            (readings[s] < MIN_SENSOR_READING ||
             readings[s] > MAX_SENSOR_RANGE   ||
             std::isnan(readings[s])          ||
             std::isinf(readings[s]))) {
            valid[s] = false;
        }
    }

    int num_valid = 0;
    for (int s = 0; s < 3; s++) num_valid += valid[s];
    if (num_valid == 0) return;

    float inv_2s2     = -0.5f / SENSOR_SIGMA_SQ;
    float outlier_pen = -12.5f;
    float max_log_w   = -1e30f;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        float h_rad = _particles[i].heading * DEG_TO_RAD;
        float sin_h = sinf(h_rad);
        float cos_h = cosf(h_rad);
        float px    = _particles[i].x;
        float py    = _particles[i].y;

        float log_l = 0.0f;

        for (int s = 0; s < 3; s++) {
            if (!valid[s]) continue;

            float wx, wy;
            body_to_world(sin_h, cos_h,
                          SENSORS[s].body_x, SENSORS[s].body_y,
                          px, py, wx, wy);

            float dx_w = SENSORS[s].dir_x * cos_h - SENSORS[s].dir_y * sin_h;
            float dy_w = SENSORS[s].dir_x * sin_h + SENSORS[s].dir_y * cos_h;

            float expected = raycast_single(wx, wy, dx_w, dy_w);
            float error    = readings[s] - expected;

            if (fabsf(error) > 5.0f * SENSOR_SIGMA)
                log_l += outlier_pen;
            else
                log_l += error * error * inv_2s2;
        }

        _particles[i].weight = log_l;
        if (log_l > max_log_w) max_log_w = log_l;
    }

    float sum_w = 0.0f;
    for (int i = 0; i < NUM_PARTICLES; i++) {
        _particles[i].weight = expf(_particles[i].weight - max_log_w);
        sum_w += _particles[i].weight;
    }

    float w_avg = sum_w / NUM_PARTICLES;
    _w_slow += ALPHA_SLOW * (w_avg - _w_slow);
    _w_fast += ALPHA_FAST * (w_avg - _w_fast);

    if (sum_w > 1e-30f) {
        float inv = 1.0f / sum_w;
        for (int i = 0; i < NUM_PARTICLES; i++)
            _particles[i].weight *= inv;
    } else {
        float w = 1.0f / NUM_PARTICLES;
        for (int i = 0; i < NUM_PARTICLES; i++)
            _particles[i].weight = w;
    }
}


void Location::resample() {
    float sum_sq = 0.0f;
    for (int i = 0; i < NUM_PARTICLES; i++)
        sum_sq += _particles[i].weight * _particles[i].weight;

    // Augmented MCL: probability of injecting random particles
    float p_random = 0.0f;
    if (_w_slow > 1e-30f)
        p_random = fmaxf(0.0f, 1.0f - _w_fast / _w_slow);

    if (sum_sq < 1e-30f) return;
    float n_eff = 1.0f / sum_sq;

    // Resample when Neff drops or when augmented MCL wants random injection
    if (p_random < 1e-6f && n_eff >= NEFF_RATIO * NUM_PARTICLES) return;

    int num_random = static_cast<int>(p_random * NUM_PARTICLES);
    if (num_random > NUM_PARTICLES) num_random = NUM_PARTICLES;
    int num_resamp = NUM_PARTICLES - num_random;

    float inv_n = 1.0f / NUM_PARTICLES;

    // Systematic low-variance resampling for the retained portion
    if (num_resamp > 0) {
        float step = 1.0f / num_resamp;
        float r = rand_uniform() * step;
        float c = _particles[0].weight;
        int   j = 0;

        for (int i = 0; i < num_resamp; i++) {
            float u = r + i * step;
            while (u > c && j < NUM_PARTICLES - 1) {
                j++;
                c += _particles[j].weight;
            }
            _resample_buf[i]        = _particles[j];
            _resample_buf[i].weight = inv_n;
        }
    }

    // Inject random particles uniformly across the whole field
    for (int i = num_resamp; i < NUM_PARTICLES; i++) {
        _resample_buf[i].x       = (rand_uniform() - 0.5f) * 2.0f
                                    * (FIELD_HALF_X - 1.0f);
        _resample_buf[i].y       = (rand_uniform() - 0.5f) * 2.0f
                                    * (FIELD_HALF_Y - 1.0f);
        _resample_buf[i].heading = wrap_heading(
                                    (rand_uniform() - 0.5f) * 360.0f);
        _resample_buf[i].weight  = inv_n;
    }

    std::memcpy(_particles, _resample_buf, sizeof(_particles));
}


Pose Location::compute_estimate() const {
    float sx = 0.0f, sy = 0.0f;
    float ss = 0.0f, sc = 0.0f;
    float sw = 0.0f;

    for (int i = 0; i < NUM_PARTICLES; i++) {
        float w = _particles[i].weight;
        sx += w * _particles[i].x;
        sy += w * _particles[i].y;
        float hr = _particles[i].heading * DEG_TO_RAD;
        ss += w * sinf(hr);
        sc += w * cosf(hr);
        sw += w;
    }

    Pose p;
    if (sw > 1e-30f) {
        float inv = 1.0f / sw;
        p.x       = static_cast<double>(sx * inv);
        p.y       = static_cast<double>(sy * inv);
        p.heading  = static_cast<double>(atan2f(ss, sc) * RAD_TO_DEG);
    } else {
        p = _pose;
    }
    return p;
}


float Location::raycast_single(float ox, float oy,
                               float dx, float dy) const {
    float min_t = MAX_SENSOR_RANGE;

    for (int i = 0; i < MAP_LINE_COUNT; i++) {
        float ax = map[i].x1;
        float ay = map[i].y1;
        float rx = map[i].x2 - ax;
        float ry = map[i].y2 - ay;
        float ex = ax - ox;
        float ey = ay - oy;

        float det = rx * dy - dx * ry;
        if (fabsf(det) < 1e-6f) continue;

        float inv = 1.0f / det;
        float t   = (rx * ey - ry * ex) * inv;
        float s   = (dx * ey - dy * ex) * inv;

        if (t >= 0.0f && s >= 0.0f && s <= 1.0f && t < min_t)
            min_t = t;
    }

    return min_t;
}


float Location::rand_uniform() {
    _rng_state ^= _rng_state << 13;
    _rng_state ^= _rng_state >> 17;
    _rng_state ^= _rng_state << 5;
    return static_cast<float>(_rng_state) * (1.0f / 4294967296.0f);
}

float Location::gaussian_noise(float sigma) {
    float sum = -6.0f;
    for (int i = 0; i < 12; i++)
        sum += rand_uniform();
    return sum * sigma;
}

} // namespace bot
