#pragma once
#include "vex.h"
#include "bot/imu.hpp"
#include "bot/piston.hpp"

namespace bot {

    extern vex::brain Brain;
    extern vex::controller Controller1;
    extern vex::competition Competition;
    
    namespace motors {
        extern vex::motor leftA; extern vex::motor leftB; extern vex::motor leftC;
        extern vex::motor_group left_dt;
        extern vex::motor rightA; extern vex::motor rightB; extern vex::motor rightC;
        extern vex::motor_group right_dt;
        extern vex::motor_group all_dt;
        extern vex::motor lower; extern vex::motor mid; extern vex::motor upper;
        extern vex::motor_group intake;

    }

    namespace sensors {
        
        extern bot::Inertial imu;
        extern vex::distance left_dist; extern vex::distance right_dist;
        extern vex::distance back_dist; extern vex::distance front_dist;
        extern vex::distance left_dist_fwd; extern vex::distance left_dist_aft;
        extern vex::distance right_dist_fwd; extern vex::distance right_dist_aft;
        extern vex::distance front_dist_left; extern vex::distance front_dist_right;
        extern vex::distance back_dist_left; extern vex::distance back_dist_right;
    }

    namespace pistons {
        extern vex::digital_out front_arm_piston;
        extern vex::digital_out back_arm_piston;
        extern bot::piston_group arm_pistons;
        extern bot::piston match_load_piston;
        extern bot::piston hood_piston;
        extern vex::digital_out mid_piston;
        extern vex::digital_out _match_load_piston;
        extern vex::digital_out _hood_piston;
        extern void toggle_arm_piston();
        extern void toggle_match_load_piston();
        extern void toggle_hood_piston();
    }
}