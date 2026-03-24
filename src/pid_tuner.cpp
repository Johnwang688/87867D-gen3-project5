#include "pid_tuner.hpp"

PIDTuner::PIDTuner(PID& pid) : 
_pid(pid), _kp(pid.get_kp()), _ki(pid.get_ki()), _kd(pid.get_kd()), _lr(0.1), 
_last_drive_direction(-1), _last_learn_direction(1), _last_time(15000001), _best_time(15000001), _best_gains(pid.get_gains()) {}

void PIDTuner::tune_drive(bot::gainType gain) {
    double start_time = vex::timer::systemHighResolution();
    bot::drivetrains::dt._drive_pid.set_gains(_kp, _ki, _kd);
    bot::drivetrains::dt.drive_for(600 * -1 * _last_drive_direction, 15000, 100, 0);
    _last_drive_direction *= -1;
    double end_time = vex::timer::systemHighResolution();
    double total_time = end_time - start_time;
    _last_time = total_time;
    if (total_time < _best_time) {
        _best_time = total_time;
        _best_gains = PIDGains{_kp, _ki, _kd};
    }
    if (total_time < _last_time) {
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * 0.1 * _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * 0.1 * _last_learn_direction;
                break;
        }
    } else {
        _last_learn_direction *= -1;
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * 0.1* _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * 0.1 * _last_learn_direction;
                break;
        }
    }
}

void PIDTuner::tune_turn(bot::gainType gain) {
    double start_time = vex::timer::systemHighResolution();
    bot::drivetrains::dt._turn_pid.set_gains(_kp, _ki, _kd);
    bot::drivetrains::dt.turn_to_heading(90 * _last_drive_direction, 15000, 100);
    _last_drive_direction *= -1;
    double end_time = vex::timer::systemHighResolution();
    double total_time = end_time - start_time;
    _last_time = total_time;
    if (total_time < _best_time) {
        _best_time = total_time;
        _best_gains = PIDGains{_kp, _ki, _kd};
    }
    if (total_time < _last_time) {
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * _last_learn_direction;
                break;
        }
    } else {
        _last_learn_direction *= -1;
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * _last_learn_direction;
                break;
        }
    }
}

void PIDTuner::tune_heading(bot::gainType gain) {
    double start_time = vex::timer::systemHighResolution();
    bot::drivetrains::dt._heading_pid.set_gains(_kp, _ki, _kd);
    bot::drivetrains::dt.drive_for(400, 15000, 60, 45 + 45 * _last_drive_direction);
    _last_drive_direction *= -1;
    double end_time = vex::timer::systemHighResolution();
    double total_time = end_time - start_time;
    _last_time = total_time;
    if (total_time < _best_time) {
        _best_time = total_time;
        _best_gains = PIDGains{_kp, _ki, _kd};
    }
    if (total_time < _last_time) {
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * _last_learn_direction;
                break;
        }
    } else {
        _last_learn_direction *= -1;
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * _last_learn_direction;
                break;
        }
    }
}

void PIDTuner::tune_distance(bot::gainType gain) {
    double start_time = vex::timer::systemHighResolution();
    bot::drivetrains::dt._dist_pid.set_gains(_kp, _ki, _kd);
    bot::drivetrains::dt.drive_dist(600 + 200*_last_drive_direction, 15000, 100, 0, 5.0, bot::sensors::back_dist, bot::rev);
    _last_drive_direction *= -1;
    double end_time = vex::timer::systemHighResolution();
    double total_time = end_time - start_time;
    _last_time = total_time;
    if (total_time < _best_time) {
        _best_time = total_time;
        _best_gains = PIDGains{_kp, _ki, _kd};
    }
    if (total_time < _last_time) {
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * _last_learn_direction;
                break;
        }
    } else {
        _last_learn_direction *= -1;
        switch (gain) {
            case bot::kp:
                _kp += _lr * _last_learn_direction;
                break;
            case bot::ki:
                _ki += _lr * _last_learn_direction;
                break;
            case bot::kd:
                _kd += _lr * _last_learn_direction;
                break;
        }
    }
}

PIDGains PIDTuner::tune(bot::PIDType type, int epochs) {
    switch (type) {
        case bot::drive:
            for (int i = 0; i < epochs; i++) {
                tune_drive(bot::kp);
            }
            for (int i = 0; i < epochs; i++) {
                tune_drive(bot::kd);
            }
            break;
        case bot::turn:
            for (int i = 0; i < epochs; i++) {
                tune_turn(bot::kp);
            }
            for (int i = 0; i < epochs; i++) {
                tune_turn(bot::kd);
            }
            break;
        case bot::heading:
            for (int i = 0; i < epochs; i++) {
                tune_heading(bot::kp);
            }
            for (int i = 0; i < epochs; i++) {
                tune_heading(bot::kd);
            }
            break;
        case bot::distance:
            for (int i = 0; i < epochs; i++) {
                tune_distance(bot::kp);
            }
            for (int i = 0; i < epochs; i++) {
                tune_distance(bot::kd);
            }
            break;
    }
    return _best_gains;
}