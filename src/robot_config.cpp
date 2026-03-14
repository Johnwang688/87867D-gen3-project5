#include "bot/robot_config.hpp"

namespace bot {

    vex::brain Brain;
    vex::controller Controller1 = vex::controller(vex::primary);
    vex::competition Competition;

    namespace motors {
        vex::motor leftA = vex::motor(vex::PORT11, vex::ratio6_1, true);
		vex::motor leftB = vex::motor(vex::PORT12, vex::ratio6_1, false);
		vex::motor leftC = vex::motor(vex::PORT20, vex::ratio6_1, true); 

		vex::motor rightA = vex::motor(vex::PORT1, vex::ratio6_1, false);
		vex::motor rightB = vex::motor(vex::PORT9, vex::ratio6_1, true);
		vex::motor rightC = vex::motor(vex::PORT10, vex::ratio6_1, false);

		vex::motor_group left_dt = vex::motor_group(leftA, leftB, leftC);
		vex::motor_group right_dt = vex::motor_group(rightA, rightB, rightC);
		vex::motor_group all_dt = vex::motor_group(leftA, leftB, leftC, rightA, rightB, rightC);

		vex::motor upper = vex::motor(vex::PORT2, vex::ratio18_1, true);
		vex::motor mid = vex::motor(vex::PORT16, vex::ratio18_1, false);
		vex::motor lower = vex::motor(vex::PORT3, vex::ratio6_1, false);
		vex::motor_group intake = vex::motor_group(upper, mid, lower);
    }

    namespace sensors {
        bot::Inertial imu(vex::PORT18);
        vex::distance left_dist = vex::distance(vex::PORT15);
        vex::distance right_dist = vex::distance(vex::PORT7);
        vex::distance back_dist = vex::distance(vex::PORT8);
		vex::distance front_dist = vex::distance(vex::PORT14);
        //vex::distance left_dist_fwd = vex::distance(vex::PORT12);
		//vex::distance left_dist_aft = vex::distance(vex::PORT13);
		//vex::distance right_dist_fwd = vex::distance(vex::PORT6);
		//vex::distance right_dist_aft = vex::distance(vex::PORT7);
		//vex::distance front_dist_left = vex::distance(vex::PORT15);
		//vex::distance front_dist_right = vex::distance(vex::PORT14);
		//vex::distance back_dist_left = vex::distance(vex::PORT17);
		//vex::distance back_dist_right = vex::distance(vex::PORT8);
    }

	namespace pistons {
		vex::digital_out match_load_piston = vex::digital_out(bot::Brain.ThreeWirePort.C);
		vex::digital_out arm_piston = vex::digital_out(bot::Brain.ThreeWirePort.F);
		void toggle_arm_piston() {
			arm_piston.set(!arm_piston.value());
		}
		void toggle_match_load_piston() {
			match_load_piston.set(!match_load_piston.value());
		}
	}
}