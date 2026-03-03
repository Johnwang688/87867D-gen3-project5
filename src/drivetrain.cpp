#include "bot/drivetrain.hpp"

namespace bot {

Drivetrain::Drivetrain(vex::motor_group& left_dt, 
    vex::motor_group& right_dt, vex::inertial& imu):
    _left_dt(left_dt),
    _right_dt(right_dt),
    _imu(imu),
    _wheel_diameter(WHEEL_DIAMETER),
    _track_width(TRACK_WIDTH),
    _gear_ratio(GEAR_RATIO),
    _max_voltage(MAX_VOLTAGE),
    _max_accel(MAX_ACCEL),
    _drive_pid(DRIVE_KP, DRIVE_KI, DRIVE_KD),
    _heading_pid(HEADING_KP, HEADING_KI, HEADING_KD),
    _turn_pid(TURN_KP, TURN_KI, TURN_KD),
    _left_arc_pid(ARC_KP, ARC_KI, ARC_KD),
    _right_arc_pid(ARC_KP, ARC_KI, ARC_KD),
    _heading_pidf(20.0, 0.0, 0.0),
    _distance_pidf(0.1, 0.0, 0.0)
    {}

void Drivetrain::tank_drive(double left_speed, double right_speed) {
    left_speed = math::clamp(left_speed, -100, 100);
    right_speed = math::clamp(right_speed, -100, 100);
    _left_dt.spin(vex::forward, left_speed * 0.11, vex::volt);
    _right_dt.spin(vex::forward, right_speed * 0.11, vex::volt);
}

void Drivetrain::arcade_drive(double fwd, double turn) {
    fwd = math::clamp(fwd, -100, 100);
    turn = math::clamp(turn, -100, 100);
    double left_speed = math::clamp(fwd + turn, -100, 100);
    double right_speed = math::clamp(fwd - turn, -100, 100);
    tank_drive(left_speed, right_speed);
}

void Drivetrain::stop() {
    _left_dt.stop();
    _right_dt.stop();
}

void Drivetrain::brake() {
    _left_dt.setStopping(vex::brakeType::brake);
    _right_dt.setStopping(vex::brakeType::brake);
}

void Drivetrain::coast() {
    _left_dt.setStopping(vex::brakeType::coast);
    _right_dt.setStopping(vex::brakeType::coast);
}

void Drivetrain::hold() {
    _left_dt.setStopping(vex::brakeType::hold);
    _right_dt.setStopping(vex::brakeType::hold);
}

void Drivetrain::drive_for(double distance, double timeout, double speed_limit, double target_heading) {
    double start_time = bot::Brain.Timer.time(vex::msec);
    int settle = 0;
    double heading_error, heading_correction, speed, current_pos, left_speed, right_speed;
    _left_dt.setPosition(0, vex::degrees);
    _right_dt.setPosition(0, vex::degrees);
    _drive_pid.reset();
    _heading_pid.reset();
    double dist = helpers::mmToDegrees(distance);
    while (timeout > 0 && bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        heading_error = helpers::angular_difference(_imu.heading(vex::degrees), target_heading);
        heading_correction = _heading_pid.compute(heading_error, 0.0, 0.02);
        current_pos = (_left_dt.position(vex::degrees) + _right_dt.position(vex::degrees)) / 2.0;
        speed = _drive_pid.compute(dist, current_pos, 0.02);
        left_speed = speed + heading_correction;
        right_speed = speed - heading_correction;
        left_speed = math::clamp(left_speed, -speed_limit, speed_limit);
        right_speed = math::clamp(right_speed, -speed_limit, speed_limit);
        left_speed *= (_max_voltage / 100.0);
        right_speed *= (_max_voltage / 100.0);
        _left_dt.spin(vex::forward, left_speed, vex::voltageUnits::volt);
        _right_dt.spin(vex::forward, right_speed, vex::voltageUnits::volt);
        if (std::abs(dist - current_pos) < 25
        && std::abs(heading_error) < 1.0) {
            settle++;
        } else {
            settle = 0;
        }
        vex::task::sleep(20);
        if (settle >= 3) break;
    }
    _left_dt.spin(vex::forward, 0, vex::voltageUnits::volt);
    _right_dt.spin(vex::forward, 0, vex::voltageUnits::volt);
}

void Drivetrain::drive(double distance, double timeout, double speed_limit, double target_heading) {
    double start_time = bot::Brain.Timer.time(vex::msec);
    double heading_error, heading_correction, current_pos, position_error, left_speed, right_speed, direction;
    _left_dt.setPosition(0, vex::degrees);
    _right_dt.setPosition(0, vex::degrees);
    _heading_pid.reset();
    double dist = helpers::mmToDegrees(distance);
    direction = dist > 0 ? 1 : -1;

    while (timeout > 0 && bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        heading_error = helpers::angular_difference(_imu.heading(vex::degrees), target_heading);
        heading_correction = _heading_pid.compute(heading_error, 0.0, 0.01);
        current_pos = (_left_dt.position(vex::degrees) + _right_dt.position(vex::degrees)) / 2.0;
        position_error = dist - current_pos;
        left_speed = direction * speed_limit + heading_correction;
        right_speed = direction * speed_limit - heading_correction;
        left_speed = math::clamp(left_speed, -speed_limit, speed_limit);
        right_speed = math::clamp(right_speed, -speed_limit, speed_limit);
        left_speed *= (_max_voltage / 100.0);
        right_speed *= (_max_voltage / 100.0);
        _left_dt.spin(vex::forward, left_speed, vex::voltageUnits::volt);
        _right_dt.spin(vex::forward, right_speed, vex::voltageUnits::volt);
 
        if (std::abs(position_error) < (distance / 10.0)) break;
        if (direction == 1) {
            if (current_pos > dist) break;
        } else {
            if (current_pos < dist) break;
        }
        vex::task::sleep(10);
    }
    _left_dt.stop();
    _right_dt.stop();
}

void Drivetrain::turn_to_heading(double heading, double timeout, double speed_limit) {
    double start_time = bot::Brain.Timer.time(vex::msec);
    int settle_count = 0;
    _left_dt.setPosition(0, vex::degrees);
    _right_dt.setPosition(0, vex::degrees);
    _turn_pid.reset();
    double current_heading, heading_error, output, left_speed, right_speed;
    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        current_heading = _imu.heading(vex::degrees);
        heading_error = helpers::angular_difference(current_heading, heading);
        output = _turn_pid.compute(heading_error, 0.0, 0.02);
        output = math::clamp(output, -speed_limit, speed_limit);
        left_speed = output * 0.4;
        right_speed = -output * 0.4;
        _left_dt.spin(vex::forward, left_speed, vex::voltageUnits::volt);
        _right_dt.spin(vex::forward, right_speed, vex::voltageUnits::volt);
        if (std::abs(heading_error) < 1.0) {
            settle_count++;
        } else {
            settle_count = 0;
        }
        vex::task::sleep(20);
        if (settle_count >= 3) break;
    }
    brake();
    _left_dt.stop();
    _right_dt.stop();
    coast();
}

void Drivetrain::drive_arc(double radius, double angle, double timeout, double speed_limit, double lookahead) {
    int settle_count = 0;
    double start_time = bot::Brain.Timer.time(vex::msec);
    double start_heading = _imu.heading(vex::degrees);
    double direction = (radius * angle >= 0) ? 1.0 : -1.0;
    double arc_length = std::abs(radius) * math::to_rad(std::abs(angle));
    double target_encoder = direction * helpers::mmToDegrees(arc_length);
    double heading_target, heading_error, heading_correction, current_pos, left_speed, right_speed, progress;
    _left_dt.setPosition(0, vex::degrees);
    _right_dt.setPosition(0, vex::degrees);
    _heading_pid.reset();
    _heading_pid.set_gains(2.0, 0.0, 1.0);
    _drive_pid.reset();
    lookahead = direction * std::abs(lookahead);
    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        current_pos = (_left_dt.position(vex::degrees) + _right_dt.position(vex::degrees)) / 2.0;
        progress = (current_pos + lookahead) / target_encoder;
        progress = math::clamp(progress, 0.0, 1.0);
        heading_target = helpers::wrapTo180(start_heading + (angle * progress));
        heading_error = helpers::angular_difference(_imu.heading(vex::degrees), heading_target);
        heading_correction = _heading_pid.compute(heading_error, 0.0, 0.02);
        left_speed = direction * speed_limit + heading_correction;
        right_speed = direction * speed_limit - heading_correction;
        left_speed = math::clamp(left_speed, -speed_limit, speed_limit);
        right_speed = math::clamp(right_speed, -speed_limit, speed_limit);
        left_speed *= (_max_voltage / 100.0);
        right_speed *= (_max_voltage / 100.0);
        _left_dt.spin(vex::forward, left_speed, vex::voltageUnits::volt);
        _right_dt.spin(vex::forward, right_speed, vex::voltageUnits::volt);
        if (std::abs(current_pos - target_encoder) < 25 && std::abs(heading_error) < 1.0) {
            settle_count++;
        } else {
            settle_count = 0;
        }
        vex::task::sleep(20);
        if (settle_count >= 3) break;
    }
    _left_dt.stop();
    _right_dt.stop();
    _heading_pid.set_gains(HEADING_KP, HEADING_KI, HEADING_KD);
    coast();
}

namespace mcl { extern bot::Location location; }

void Drivetrain::pure_pursuit(const std::vector<PathPoint>& path, double lookahead_dist,
                              double base_speed, double timeout) {
    if (path.size() < 2) return;

    double start_time = bot::Brain.Timer.time(vex::msec);
    const double end_threshold = 30.0;
    const double decel_radius = lookahead_dist * 1.5;
    const double min_speed = 20.0;
    size_t search_start = 0;
    int settle_count = 0;

    // Heading PID with derivative damping to prevent overshoot oscillation.
    // The D term acts as a brake on heading rate-of-change.
    PID pp_heading(PP_KP, PP_KI, PP_KD);

    // Virtual target beyond the last waypoint so the robot always has a
    // point ahead to steer toward, preventing erratic turns at the end.
    double ext_x = path.back().x;
    double ext_y = path.back().y;
    {
        size_t n = path.size();
        double seg_dx = path[n - 1].x - path[n - 2].x;
        double seg_dy = path[n - 1].y - path[n - 2].y;
        double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
        if (seg_len > 1.0) {
            ext_x += (seg_dx / seg_len) * lookahead_dist;
            ext_y += (seg_dy / seg_len) * lookahead_dist;
        }
    }

    while (bot::Brain.Timer.time(vex::msec) - start_time < timeout) {
        Pose pose = bot::mcl::location.get_pose();

        // Find closest point forward on path (never backtracks)
        size_t closest = search_start;
        double best_dsq = 1e30;
        for (size_t i = search_start; i < path.size(); i++) {
            double dxi = path[i].x - pose.x;
            double dyi = path[i].y - pose.y;
            double dsq = dxi * dxi + dyi * dyi;
            if (dsq < best_dsq) {
                best_dsq = dsq;
                closest = i;
            }
        }
        search_start = closest;

        double dx_end = path.back().x - pose.x;
        double dy_end = path.back().y - pose.y;
        double dist_to_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);

        if (dist_to_end < end_threshold) {
            settle_count++;
            if (settle_count >= 3) break;
        } else {
            settle_count = 0;
        }

        // Lookahead: first path point >= Ld away from robot.
        // If every remaining point is inside the lookahead circle, fall
        // back to the virtual extension point so steering stays stable.
        double target_x, target_y;
        size_t la = path.size() - 1;
        bool found_la = false;
        double la_sq = lookahead_dist * lookahead_dist;
        for (size_t i = closest; i < path.size(); i++) {
            double dxi = path[i].x - pose.x;
            double dyi = path[i].y - pose.y;
            if (dxi * dxi + dyi * dyi >= la_sq) {
                la = i;
                found_la = true;
                break;
            }
        }

        if (found_la) {
            target_x = path[la].x;
            target_y = path[la].y;
        } else {
            target_x = ext_x;
            target_y = ext_y;
        }

        std::int8_t dir = path[la].direction;
        if (dir != 1 && dir != -1) dir = 1;

        // Desired heading toward the target (MCL convention: 0°=+Y, CCW+)
        double dx = target_x - pose.x;
        double dy = target_y - pose.y;
        double desired_heading = std::atan2(-dx, dy) * 180.0 / M_PI;
        if (dir == -1) {
            desired_heading = helpers::wrapTo180(desired_heading + 180.0);
        }

        // PID heading correction — D term dampens heading rate to kill oscillation
        double heading_error = helpers::angular_difference(desired_heading, pose.heading);
        double heading_correction = pp_heading.compute(heading_error, 0.0, 0.02);
        double max_correction = base_speed * MAX_CORRECTION;
        heading_correction = math::clamp(heading_correction, -max_correction, max_correction);

        // Decelerate near the final waypoint
        double speed_scale = 1.0;
        if (dist_to_end < decel_radius) {
            speed_scale = (min_speed / base_speed)
                        + (1.0 - min_speed / base_speed) * (dist_to_end / decel_radius);
        }

        double speed = static_cast<double>(dir) * base_speed * speed_scale;
        double left_speed = speed + heading_correction;
        double right_speed = speed - heading_correction;

        double speed_lim = speed * speed_scale;
        left_speed = math::clamp(left_speed, -speed_lim, speed_lim);
        right_speed = math::clamp(right_speed, -speed_lim, speed_lim);

        _left_dt.spin(vex::forward, left_speed * (_max_voltage / 100.0), vex::voltageUnits::volt);
        _right_dt.spin(vex::forward, right_speed * (_max_voltage / 100.0), vex::voltageUnits::volt);

        vex::task::sleep(20);
    }

    _left_dt.stop();
    _right_dt.stop();
}

void Drivetrain::drive_to(std::vector<Waypoint> waypoints, double speed_limit){
    double start_time = bot::Brain.Timer.time(vex::msec);
    const double kF = 50.0;
    const double max_heading_correction = 0.5;
    Waypoint start_waypoint;
    start_waypoint.x = 0; start_waypoint.y = 0;
    start_waypoint.heading = _imu.heading(vex::degrees);
    start_waypoint.time = start_time;
    for (Waypoint& waypoint : waypoints) {
        double cycles = (waypoint.time) / 20.0f;
        double current_encoder = (_left_dt.position(vex::degrees) + _right_dt.position(vex::degrees)) / 2.0f;
        double current_heading = _imu.heading(vex::degrees);
        double waypoint_heading_error = helpers::angular_difference(current_heading, waypoint.heading);
        double dx = waypoint.x - start_waypoint.x;
        double dy = waypoint.y - start_waypoint.y;
        double arc_distance;
        double chord = std::hypot(dx, dy);
        if (std::abs(waypoint_heading_error) < 1e-6) {
            arc_distance = helpers::mmToDegrees(chord);
        } else {
            double theta = math::to_rad(std::abs(waypoint_heading_error));
            arc_distance = helpers::mmToDegrees((chord * theta) / (2.0 * std::sin(theta * 0.5)));
        }
        std::vector<double> headings;
        std::vector<double> distances;
        headings.reserve(static_cast<size_t>(cycles));
        distances.reserve(static_cast<size_t>(cycles));
        double inv_cycles = 1.0 / cycles;
        for (int i = 0; i < cycles; i++) {
            double progress = (i + 1) * inv_cycles;
            headings.push_back(helpers::wrapTo180(current_heading + (waypoint_heading_error * progress)));
        }
        double direction;
        switch (waypoint.direction) {
            case bot::fwd:
                direction = 1.0;
                break;
            case bot::rev:
                direction = -1.0;
                break;
        }
        double signed_arc_distance = arc_distance * direction;

        for (int i = 0; i < cycles; i++) {
            double progress = (i + 1) * inv_cycles;
            distances.push_back(current_encoder + (signed_arc_distance * progress));
        }
        _heading_pidf.reset();
        _distance_pidf.reset();
        double heading_error, distance_error, heading_correction, distance_correction, feedforward;
        double base_speed, heading_ratio, scaled_heading, left_speed, right_speed;
        for (int i = 0; i < cycles; i++) {
            heading_error = helpers::angular_difference(_imu.heading(vex::degrees), headings[i]);
            distance_error = distances[i] - (_left_dt.position(vex::degrees) + _right_dt.position(vex::degrees)) / 2.0f;
            heading_correction = _heading_pidf.compute(heading_error, 0.0, 0.02, 5.0);
            distance_correction = _distance_pidf.compute(distance_error, 0.0, 0.02);
            distance_correction = math::clamp(distance_correction, -speed_limit + kF, speed_limit - kF);
            feedforward = kF * direction;
            base_speed = feedforward + distance_correction;
            heading_ratio = math::clamp(heading_correction * 0.01, -max_heading_correction, max_heading_correction);
            scaled_heading = heading_ratio * std::max(std::abs(base_speed), 10.0);
            left_speed = base_speed + scaled_heading;
            right_speed = base_speed - scaled_heading;
            left_speed = math::clamp(left_speed, -speed_limit, speed_limit);
            right_speed = math::clamp(right_speed, -speed_limit, speed_limit);
            left_speed *= (_max_voltage / 100.0);
            right_speed *= (_max_voltage / 100.0);
            _left_dt.spin(vex::forward, left_speed, vex::voltageUnits::volt);
            _right_dt.spin(vex::forward, right_speed, vex::voltageUnits::volt);
            vex::task::sleep(20);
        }

        start_waypoint = waypoint;
    }
    _left_dt.stop();
    _right_dt.stop();
    coast();
}
}
