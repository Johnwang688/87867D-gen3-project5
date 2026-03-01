#pragma once
#include "bot/types.hpp"

constexpr double PI = 3.14159265358979323846;

// PID constants
constexpr double DRIVE_KP = 0.20;
constexpr double DRIVE_KI = 0.01;
constexpr double DRIVE_KD = 0.02;

constexpr double TURN_KP = 0.40;
constexpr double TURN_KI = 0.0;
constexpr double TURN_KD = 0.03;

constexpr double ARC_KP = 0.08;
constexpr double ARC_KI = 0.01;
constexpr double ARC_KD = 0.005;

constexpr double MAX_INTEGRAL = 1000.0;
constexpr double MAX_OUTPUT = 100.0;
constexpr double DT = 0.02; // seconds 

// pure pursuit constants
constexpr double HEADING_KP = 1.25;
constexpr double HEADING_KI = 0.05;
constexpr double HEADING_KD = 0.02;


constexpr double K_SLOW = 0.40;
constexpr double MIN_SPEED_PCT = 0.1;
constexpr double MAX_CURV_SPEED_FACTOR = 0.7;
constexpr double PP_DT = 0.05;

//mm
constexpr double WHEEL_DIAMETER = 88.25;
constexpr double TRACK_WIDTH = 300.0;
constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

// Gear ratio is output speed / input speed
constexpr double GEAR_RATIO = 450.0/600.0;
constexpr double MM_PER_TICK = (WHEEL_CIRCUMFERENCE / 360.0) * GEAR_RATIO;

// Maximum voltage to apply to motors
constexpr double MAX_VOLTAGE = 11.0;

// controller deadzone 
constexpr double CONTROLLER_DEADZONE = 1.0;
constexpr double MID_SPEED = 80.0;

// max accel gain for PID controllers 
constexpr double MAX_ACCEL = 3.5;

// driver sens constants
constexpr double A = 2.0;
constexpr double B = 1.055;
constexpr double C = -A;

// MCL particle filter constants
constexpr int MCL_NUM_PARTICLES = 200;

// MCL sensor positions in robot body frame (mm)
// Body frame: +X = right, +Y = forward, origin at robot center
constexpr float LEFT_SENSOR_BODY_X  = -135.0f;
constexpr float LEFT_SENSOR_BODY_Y  =   88.0f;
constexpr float RIGHT_SENSOR_BODY_X =  146.0f;
constexpr float RIGHT_SENSOR_BODY_Y =   92.0f;
constexpr float BACK_SENSOR_BODY_X  =    0.0f;
constexpr float BACK_SENSOR_BODY_Y  = -130.0f;

// Robot dimensions and offsets (in mm)
constexpr std::int16_t LEFT_FORWARD_OFFSET_X = -135;
constexpr std::int16_t LEFT_FORWARD_OFFSET_Y = 88;
constexpr std::int16_t RIGHT_FORWARD_OFFSET_X = 146;
constexpr std::int16_t RIGHT_FORWARD_OFFSET_Y = 92;

constexpr std::int16_t LEFT_AFT_OFFSET_X = -141;
constexpr std::int16_t LEFT_AFT_OFFSET_Y = 10;
constexpr std::int16_t RIGHT_AFT_OFFSET_X = 135;
constexpr std::int16_t RIGHT_AFT_OFFSET_Y = -30;

constexpr std::int16_t FRONT_LEFT_OFFSET_X = -133;
constexpr std::int16_t FRONT_LEFT_OFFSET_Y = 155;
constexpr std::int16_t FRONT_RIGHT_OFFSET_X = 133;
constexpr std::int16_t FRONT_RIGHT_OFFSET_Y = 155;

constexpr std::int16_t BACK_LEFT_OFFSET_X = -100;
constexpr std::int16_t BACK_LEFT_OFFSET_Y = -134;
constexpr std::int16_t BACK_RIGHT_OFFSET_X = 100;
constexpr std::int16_t BACK_RIGHT_OFFSET_Y = -134;

// field dimensions (in mm)
constexpr std::int16_t WIDTH = 3560;
constexpr std::int16_t HEIGHT = 3560;
constexpr std::int16_t OUT_OF_BOUNDS = WIDTH/2;

// All units in mm. Origin (0,0) is center.
constexpr Line map[] = {
    // 1. Perimeter Walls
    {-WIDTH/2, -HEIGHT/2,  WIDTH/2, -HEIGHT/2}, { WIDTH/2, -HEIGHT/2,  WIDTH/2,  HEIGHT/2},
    { WIDTH/2,  HEIGHT/2, -WIDTH/2,  HEIGHT/2}, {-WIDTH/2,  HEIGHT/2, -WIDTH/2, -HEIGHT/2},

    // 2. center goal
    {-70, 0, 0, -70}, {0, 70, 70, 0}, {-120, -120, 120, 120},


    // left long goal 
    {-1200, -510, -1150, -600}, {-1200, -510, -1250, -600},
    {-1200, 510, -1150, 600}, {-1200, 510, -1250, 600},
    // bottom long goal
    //{-510, -1200,-600,-1130}, {-510, -1200,-600,-1270},
    //{600,-1130,510,-1200}, {600,-1270,510,-1200},


    //right long goal
    {1200, -510, 1150, -600}, {1200, -510, 1250, -600},
    {1200, 510, 1150, 600}, {1200, 510, 1250, 600},
    //top long goal
    //{600,1130,510,1200}, {600,1270,510,1200},
    //{-510,1200,-600,1130}, {-510,1200,-600,1270},

};

constexpr int MAP_LINE_COUNT = sizeof(map) / sizeof(map[0]);