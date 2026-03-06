#include "bot/constants.hpp"
#include <cmath>

const double PI = 3.14159265358979323846;

// PID constants
const double DRIVE_KP = 0.20;
const double DRIVE_KI = 0.01;
const double DRIVE_KD = 0.02;

const double TURN_KP = 0.40;
const double TURN_KI = 0.0;
const double TURN_KD = 0.03;

const double ARC_KP = 0.08;
const double ARC_KI = 0.01;
const double ARC_KD = 0.005;

const double MAX_INTEGRAL = 1000.0;
const double MAX_OUTPUT = 100.0;
const double DT = 0.02; // seconds

// pure pursuit constants
const double HEADING_KP = 1.25;
const double HEADING_KI = 0.05;
const double HEADING_KD = 0.02;

const double PP_KP = 2.0;
const double PP_KI = 0.0;
const double PP_KD = 0.3;
const double MAX_CORRECTION = 0.6;
const double MIN_TURN_SPEED_FRAC = 0.4;

// unused pp constants (from older version of pp)
const double K_SLOW = 0.40;
const double MIN_SPEED_PCT = 0.1;
const double MAX_CURV_SPEED_FACTOR = 0.7;
const double PP_DT = 0.05;

// mm
const double WHEEL_DIAMETER = 88.25;
const double TRACK_WIDTH = 300.0;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;

const double GEAR_RATIO = 450.0 / 600.0; // driving to driven. 
const double MM_PER_TICK = (WHEEL_CIRCUMFERENCE / 360.0) * GEAR_RATIO;

const double MAX_VOLTAGE = 11.0;

// controller deadzone
const double CONTROLLER_DEADZONE = 1.0;
const double MID_SPEED = 80.0;

// max accel gain for PID controllers
const double MAX_ACCEL = 3.5;

// driver sens constants
const double A = 2.0;
const double B = 1.055;
const double C = -A;

// MCL sensor positions in robot body frame (mm)
const float LEFT_SENSOR_BODY_X  = -141.0f;
const float LEFT_SENSOR_BODY_Y  =   10.0f;
const float RIGHT_SENSOR_BODY_X =  135.0f;
const float RIGHT_SENSOR_BODY_Y =   -30.0f;
const float BACK_SENSOR_BODY_X  =  100.0f;
const float BACK_SENSOR_BODY_Y  = -134.0f;

// Robot dimensions and offsets (in mm)
const std::int16_t LEFT_FORWARD_OFFSET_X = -135;
const std::int16_t LEFT_FORWARD_OFFSET_Y = 88;
const std::int16_t RIGHT_FORWARD_OFFSET_X = 146;
const std::int16_t RIGHT_FORWARD_OFFSET_Y = 92;

const std::int16_t LEFT_AFT_OFFSET_X = -141;
const std::int16_t LEFT_AFT_OFFSET_Y = 10;
const std::int16_t RIGHT_AFT_OFFSET_X = 135;
const std::int16_t RIGHT_AFT_OFFSET_Y = -30;

const std::int16_t FRONT_LEFT_OFFSET_X = -133;
const std::int16_t FRONT_LEFT_OFFSET_Y = 155;
const std::int16_t FRONT_RIGHT_OFFSET_X = 133;
const std::int16_t FRONT_RIGHT_OFFSET_Y = 155;

const std::int16_t BACK_LEFT_OFFSET_X = -100;
const std::int16_t BACK_LEFT_OFFSET_Y = -134;
const std::int16_t BACK_RIGHT_OFFSET_X = 100;
const std::int16_t BACK_RIGHT_OFFSET_Y = -134;

// field dimensions (in mm)
const std::int16_t WIDTH = 3560;
const std::int16_t HEIGHT = 3560;
const std::int16_t OUT_OF_BOUNDS = WIDTH / 2;

// All units in mm. Origin (0,0) is center.
const Line map[] = {
    // 1. Perimeter Walls
    {-WIDTH/2, -HEIGHT/2,  WIDTH/2, -HEIGHT/2}, { WIDTH/2, -HEIGHT/2,  WIDTH/2,  HEIGHT/2},
    { WIDTH/2,  HEIGHT/2, -WIDTH/2,  HEIGHT/2}, {-WIDTH/2,  HEIGHT/2, -WIDTH/2, -HEIGHT/2},
   // 2. center goal
    {-70, 0, 0, -70}, {0, 70, 70, 0}, {-120, -120, 120, 120},

    // left long goal
    {-1200, -510, -1150, -600}, {-1200, -510, -1250, -600},
    {-1200, 510, -1150, 600}, {-1200, 510, -1250, 600},

    // right long goal
    {1200, -510, 1150, -600}, {1200, -510, 1250, -600},
    {1200, 510, 1150, 600}, {1200, 510, 1250, 600},

};

const int MAP_LINE_COUNT = sizeof(map) / sizeof(map[0]);
