#pragma once

#include "bot/bot.hpp"
#include "bot/pid.hpp"
#include "location.hpp"
#include "bot/types.hpp"

namespace bot {

class Drivetrain {
    public:
        Drivetrain(
            vex::motor_group& left_dt,
            vex::motor_group& right_dt,
            vex::inertial& imu
        );
        void tank_drive(double left_speed, double right_speed);
        void arcade_drive(double fwd, double turn);
        void stop();
        void brake();
        void coast();
        void hold();

        void drive_for(double distance, double timeout, double speed_limit, double target_heading);
        void drive(double distance, double timeout, double speed_limit, double target_heading);
        void turn_to_heading(double heading, double timeout, double speed_limit);
        void drive_dist(double target_distance, double timeout, double speed_limit, double target_heading, 
            double tolerance, vex::distance& dist_sensor, bot::driveDirection direction);
        void drive_arc(double radius, double angle, double timeout, double speed_limit, double lookahead);

        void drive_to(std::vector<Waypoint> waypoints, double speed_limit);

        // All distances in mm.  base_speed is a percentage (0-100).
        void pure_pursuit(const std::vector<PathPoint>& path, double lookahead_dist,
                          double base_speed, double timeout);
    private:
        vex::motor_group& _left_dt;
        vex::motor_group& _right_dt;
        vex::inertial& _imu;
        double _wheel_diameter;
        double _track_width;
        double _gear_ratio;
        double _max_voltage;
        double _max_accel;
        PID _drive_pid;
        PID _heading_pid;
        PID _turn_pid;
        PID _left_arc_pid;
        PID _right_arc_pid;
        PID _heading_pidf;
        PID _distance_pidf;
};

}