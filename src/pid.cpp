#include "bot/pid.hpp"

PID::PID(double kp, double ki, double kd) :
    _kp(kp), _ki(ki), _kd(kd), 
    _prev_error(0), _integral(0), 
    _max_integral(1000), _max_output(100) {}

double PID::compute(double setpoint, double input, double dt) {
    double error = setpoint - input;
    _integral += error * dt;
    _integral = std::min(std::max(_integral, -_max_integral), _max_integral);
    double derivative = (error - _prev_error) / dt;
    _prev_error = error;
    double output = _kp * error + _ki * _integral + _kd * derivative;
    output = std::min(std::max(output, -_max_output), _max_output);
    return output;
}

double PID::compute(double setpoint, double input, double dt, double integral_zone) {
    double error = setpoint - input;
    if (fabs(error) < integral_zone) _integral += error * dt;
    else _integral = 0.0f;
    _integral = std::min(std::max(_integral, -_max_integral), _max_integral);
    double derivative = (error - _prev_error) / dt;
    _prev_error = error;
    double output = _kp * error + _ki * _integral + _kd * derivative;
    output = std::min(std::max(output, -_max_output), _max_output);
    return output;
}