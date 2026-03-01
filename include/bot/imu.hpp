#pragma once

#include "vex.h"
#include <cstdint>

namespace bot {

enum RotationDirection : std::int8_t {
    CW = 0,
    CCW = 1,
    Y = 2,
    X = 3
};

class Inertial : public vex::inertial {
    public: 
        Inertial(const int port) : vex::inertial(port) {}
        using vex::inertial::inertial;

        inline void calibrate() {
            vex::inertial::calibrate();
            while (this->isCalibrating()) {
                vex::task::sleep(10);
            }
            this->setHeading(0, vex::degrees);
            _math_offset = 90.0;
        }

        inline void reset() {
            this->setHeading(0, vex::degrees);
        }

        inline void set_heading(double heading){
            if (heading < 0) heading += 360;
            if (heading == 360) heading = 0;
            this->setHeading(heading, vex::degrees);
        }

        inline double get_heading() {
            return this->heading(vex::degrees);
        }

        // Math convention: CCW positive, +X axis = 0 degrees, range (-180, 180]
        inline double get_heading_math() {
            double h = _math_offset - this->heading(vex::degrees);
            while (h > 180.0) h -= 360.0;
            while (h <= -180.0) h += 360.0;
            return h;
        }

        // Sets math-convention offset so current raw reading maps to the given heading
        inline void set_heading_math(double heading) {
            _math_offset = heading + this->heading(vex::degrees);
        }

        inline double get_heading(RotationDirection direction) {
            return this->get_heading(direction, bot::RotationDirection::Y);
        }
        inline double get_heading(RotationDirection direction, RotationDirection zero) {
            switch (direction) {
                case CW:
                    switch (zero) {
                        case Y: {
                            double heading = this->heading(vex::degrees);
                            if (heading > 180) heading -= 360;
                            return heading;
                        }
                        case X: {
                            double heading = this->heading(vex::degrees) - 90;
                            if (heading > 180) heading -= 360;
                            return heading;
                        }
                        default: {
                            double heading = this->heading(vex::degrees);
                            if (heading > 180) heading -= 360;
                            return heading;
                        }
                    }
                case CCW:
                    switch (zero) {
                        case Y: {
                            double heading = 360 - this->heading(vex::degrees);
                            if (heading > 180) heading -= 360;
                            return heading;
                        }
                        case X: {
                            double heading = 90 - this->heading(vex::degrees);
                            if (heading <= -180) heading += 360;
                            return heading;
                        }
                        default: {
                            double heading = 360 - this->heading(vex::degrees);
                            if (heading > 180) heading -= 360;
                            return heading;
                        }
                    }
                default: {
                    double heading = this->heading(vex::degrees);
                    if (heading > 180) heading -= 360;
                    return heading;
                }
            }
        }
        inline double get_roll() {
            return this->orientation(vex::roll, vex::degrees);
        }
        inline double get_pitch() {
            return this->orientation(vex::pitch, vex::degrees);
        }

    private:
        // Offset for math convention heading. Default 90.0 assumes calibration facing +Y.
        double _math_offset = 90.0;
};
}
