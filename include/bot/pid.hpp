#pragma once

#include "bot/types.hpp"
#include <cmath>
#include <algorithm>

class PID {
    public:
        PID(double kp, double ki, double kd);
        double compute(double setpoint, double input, double dt);
        double compute(double setpoint, double input, double dt, double integral_zone);
        inline PIDGains get_gains() const {
            return PIDGains{_kp, _ki, _kd};
        }
        inline double get_kp() const { return _kp; }
        inline double get_ki() const { return _ki; }
        inline double get_kd() const { return _kd; }
        inline void set_gains(double kp, double ki, double kd) {
            _kp = kp;
            _ki = ki;
            _kd = kd;
        }
        inline void reset() {
            _prev_error = 0;
            _integral = 0;
        }
    private:
        double _kp;
        double _ki;
        double _kd;
        double _prev_error;
        double _integral;
        double _max_integral;
        double _max_output;
};