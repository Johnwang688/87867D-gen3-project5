#pragma once
#include "bot/types.hpp"

extern const double PI;

// PID constants
extern const double DRIVE_KP;
extern const double DRIVE_KI;
extern const double DRIVE_KD;

extern const double TURN_KP;
extern const double TURN_KI;
extern const double TURN_KD;

extern const double ARC_KP;
extern const double ARC_KI;
extern const double ARC_KD;

extern const double MAX_INTEGRAL;
extern const double MAX_OUTPUT;
extern const double DT;

// pure pursuit constants
extern const double HEADING_KP;
extern const double HEADING_KI;
extern const double HEADING_KD;

extern const double PP_KP;
extern const double PP_KI;
extern const double PP_KD;
extern const double MAX_CORRECTION;
extern const double MIN_TURN_SPEED_FRAC;

extern const double K_SLOW;
extern const double MIN_SPEED_PCT;
extern const double MAX_CURV_SPEED_FACTOR;
extern const double PP_DT;

//drivetrain feedforward constants
extern const double DRIVE_KS;
extern const double DRIVE_KV;
extern const double DRIVE_KA;
extern const double DRIVE_MAX_VEL;

// turn feedforward constants
extern const double TURN_KV;
extern const double TURN_KA;
extern const double TURN_MAX_VEL;

// centripetal acceleration limit (mm/s^2)
extern const double MAX_LATERAL_ACCEL;

//ramsete feedforward constants
extern const double KS;
extern const double KA;
extern const double KV;

// mm
extern const double WHEEL_DIAMETER;
extern const double TRACK_WIDTH;
extern const double WHEEL_CIRCUMFERENCE;

extern const double GEAR_RATIO;
extern const double MM_PER_TICK;

extern const double MAX_VOLTAGE;

// controller deadzone
extern const double CONTROLLER_DEADZONE;
extern const double MID_SPEED;

// max accel gain for PID controllers
extern const double MAX_ACCEL;

// driver sens constants
extern const double A;
extern const double B;
extern const double C;

// MCL particle filter constants
constexpr int MCL_NUM_PARTICLES = 500;

// MCL sensor positions in robot body frame (mm)
// Body frame: +X = right, +Y = forward, origin at robot center
extern const float LEFT_SENSOR_BODY_X;
extern const float LEFT_SENSOR_BODY_Y;
extern const float RIGHT_SENSOR_BODY_X;
extern const float RIGHT_SENSOR_BODY_Y;
extern const float BACK_SENSOR_BODY_X;
extern const float BACK_SENSOR_BODY_Y;

// Robot dimensions and offsets (in mm)
extern const std::int16_t LEFT_FORWARD_OFFSET_X;
extern const std::int16_t LEFT_FORWARD_OFFSET_Y;
extern const std::int16_t RIGHT_FORWARD_OFFSET_X;
extern const std::int16_t RIGHT_FORWARD_OFFSET_Y;

extern const std::int16_t LEFT_AFT_OFFSET_X;
extern const std::int16_t LEFT_AFT_OFFSET_Y;
extern const std::int16_t RIGHT_AFT_OFFSET_X;
extern const std::int16_t RIGHT_AFT_OFFSET_Y;

extern const std::int16_t FRONT_LEFT_OFFSET_X;
extern const std::int16_t FRONT_LEFT_OFFSET_Y;
extern const std::int16_t FRONT_RIGHT_OFFSET_X;
extern const std::int16_t FRONT_RIGHT_OFFSET_Y;

extern const std::int16_t BACK_LEFT_OFFSET_X;
extern const std::int16_t BACK_LEFT_OFFSET_Y;
extern const std::int16_t BACK_RIGHT_OFFSET_X;
extern const std::int16_t BACK_RIGHT_OFFSET_Y;

// field dimensions (in mm)
extern const std::int16_t WIDTH;
extern const std::int16_t HEIGHT;
extern const std::int16_t OUT_OF_BOUNDS;

// field map line segments
extern const Line map[];
extern const int MAP_LINE_COUNT;
