#pragma once

#include <cstdint>
#include <vector>

struct Line {
    float x1, y1, x2, y2;
};

struct Particle {
    float x, y;       // position in mm (field frame, origin at center)
    float heading;     // degrees, CCW positive, +Y = 0 (IMU convention)
    float weight;
};

struct Pose {
    double x, y, heading;
};

enum class IntakeMode : std::int8_t {
    STOP = 0,
    INTAKE = 1,
    OUTTAKE = -1,
};

namespace bot {
    enum driveDirection : std::int8_t {
        fwd = 1,
        rev = -1
    };
}

struct Waypoint {
    bot::driveDirection direction;
    double x;
    double y;
    double heading;
    double time;
};

struct PathPoint {
    double x, y;
};

