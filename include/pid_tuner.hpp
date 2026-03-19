#pragma once

#include "bot/bot.hpp"
#include "auton.hpp"

namespace bot {

enum PIDType {
    drive,
    turn,
    heading,
    distance
};

enum gainType {
    kp,
    ki,
    kd
};

}

class PIDTuner {
    public: 
        PIDTuner(PID& pid);
        void reset() {
            _kp = _pid.get_kp();
            _ki = _pid.get_ki();
            _kd = _pid.get_kd();
            _lr = 0.1;
            _best_gains = _pid.get_gains();
            _best_time = 15000001;
            _last_time = 15000001;
            _last_learn_direction = 1;
            _last_drive_direction = -1;
        }

        void tune_drive(bot::gainType gain);
        void tune_turn(bot::gainType gain);
        void tune_heading(bot::gainType gain);
        void tune_distance(bot::gainType gain);

        PIDGains tune(bot::PIDType type, int epochs);

        void set_learning_rate(double lr) { _lr = lr; }
        double get_learning_rate() { return _lr; }
        
        void set_gains(PIDGains gains) {
            _kp = gains.kp;
            _ki = gains.ki;
            _kd = gains.kd;
        }
        void set_gains(double kp, double ki, double kd) {
            _kp = kp;
            _ki = ki;
            _kd = kd;
        }

        void set_kp(double kp) { _kp = kp; }
        void set_ki(double ki) { _ki = ki; }
        void set_kd(double kd) { _kd = kd; }

        double get_best_time() { return _best_time; }
        PIDGains get_best_gains() { return _best_gains; }

    private:
        PID& _pid;
        double _kp;
        double _ki;
        double _kd;
        double _lr;
        double _last_drive_direction;
        double _last_learn_direction;
        double _last_time;
        double _best_time;
        PIDGains _best_gains;


};