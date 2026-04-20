#include "buttons.hpp"
#include <vector>

namespace bot {
    bool mid_scoring_status = false;
    volatile double stall_threshold = 0.35;
    volatile bool upper_roller_stall = false;
    void display_temperature() {
        double max_left_temp = 0.0, max_right_temp = 0.0;
        // list of left drivetrain motors
        std::vector<double> left_motors = {
          bot::motors::leftA.temperature(),
          bot::motors::leftB.temperature(),
          bot::motors::leftC.temperature()
        };
        // find which motor has the highest temperature
        for (int i = 0; i < left_motors.size(); i++) {
          max_left_temp = std::fmax(max_left_temp, left_motors[i]);
        }
        
        // list of right drivetrain motors
        std::vector<double> right_motors = {
          bot::motors::rightA.temperature(),
          bot::motors::rightB.temperature(), 
          bot::motors::rightC.temperature()
        };
        // find which motor has the highest temperature
        for (int i = 0; i < right_motors.size(); i++) {
          max_right_temp = std::fmax(max_right_temp, right_motors[i]);
        }
        
        //intake motor temperatures
        double lower_temp = bot::motors::lower.temperature();
        double upper_temp = bot::motors::upper.temperature();
        // display temperatures on the brain screen
        bot::Controller1.Screen.clearScreen();
        bot::Controller1.Screen.setCursor(1,1);
        bot::Controller1.Screen.print("Left Max: %.1f%%", max_left_temp);
        bot::Controller1.Screen.setCursor(2,1);
        bot::Controller1.Screen.print("Right Max: %.1f%%", max_right_temp);
        bot::Controller1.Screen.setCursor(3,1);
        bot::Controller1.Screen.print("L: %.1f U: %.1f", lower_temp, upper_temp);
    }

    namespace intake_methods {
        using namespace bot::motors;
        using namespace vex;
        double upper_roller_torque = 0.0;
        volatile bool intake_active = false;
        volatile bool intake_running = false;
        volatile bool score_upper_active = false;
        volatile bool score_upper_running = false;
        volatile bool outtake_active = false;
        volatile bool outtake_running = false;

        static void run_upper_with_direction() {
            if (bot::upper_roller_direction) {
                upper.spin(forward, 6.0, vex::volt);
            } else {
                upper.spin(reverse, 7.0, vex::volt);
            }
        }

        void intake(){
            bot::stall_threshold = 0.35;
            intake_running = true;
            lower.spin(forward, 100, percent);
            bot::upper_roller_stall = false;
            while (intake_active) {
                upper_roller_torque = upper.torque(vex::torqueUnits::Nm);
                if (upper_roller_torque > bot::stall_threshold /*|| upper_roller_current > 0.63*/) {
                    bot::upper_roller_stall = true;
                    bot::stall_threshold = 0.1;
                }
                if (bot::upper_roller_stall) {
                    upper.stop();
                    vex::this_thread::sleep_for(500);
                    if (!intake_active) {
                        break;
                    }
                    run_upper_with_direction();
                    bot::upper_roller_stall = false;
                } else {
                    run_upper_with_direction();
                }
                vex::this_thread::sleep_for(50);
            }
            intake_active = false;
            intake_running = false;
            upper.stop();
            lower.stop();
        }
        void stop_intaking(){
            intake_active = false;
            lower.stop();
            upper.stop();
        }
        void score_upper(){
            score_upper_running = true;
            while (score_upper_active) {
                if (bot::upper_roller_direction) {
                    upper.spin(forward, 100, percent);
                } else {
                    upper.spin(reverse, 100, percent);
                }
                lower.spin(forward, 100, percent);
                vex::this_thread::sleep_for(20);
            }
            score_upper_active = false;
            score_upper_running = false;
            upper.stop();
            lower.stop();
        }
        void stop_scoring_upper(){
            score_upper_active = false;
            upper.stop();
            lower.stop();
        }
        void outtake(){
            upper.spin(vex::reverse, 100, vex::percent);
            lower.spin(vex::reverse, 100, vex::percent);
            outtake_active = true;
        }
        void stop_outtaking(){
            upper.stop();
            lower.stop();
            outtake_active = false;
        }

        void score_middle(){
            bot::mid_scoring_status = true;
            upper.spin(reverse, 100.0, percent); 
        }

        void stop_scoring_middle(){
            bot::mid_scoring_status = false;
            upper.stop();
        }

        void toggle_middle_score(){
            if (bot::mid_scoring_status){
                bot::intake_methods::stop_scoring_middle();
            } else {
                bot::intake_methods::score_middle();
            }
        }
    }

    namespace buttons {

        using namespace bot::intake_methods;
        using namespace bot::pistons;

        static ButtonThread button_threads{nullptr, nullptr, nullptr, nullptr};

        static void cleanup_thread(vex::thread*& thread, volatile bool& running_flag) {
            if (thread != nullptr && !running_flag) {
                delete thread;
                thread = nullptr;
            }
        }

        static void stop_r1_thread() {
            bot::intake_methods::stop_intaking();
            cleanup_thread(button_threads.R1, bot::intake_methods::intake_running);
        }

        static void stop_r2_thread() {
            bot::intake_methods::stop_scoring_upper();
            cleanup_thread(button_threads.R2, bot::intake_methods::score_upper_running);
        }

        static void stop_l2_thread() {
            bot::intake_methods::stop_scoring_upper();
            cleanup_thread(button_threads.L2, bot::intake_methods::score_upper_running);
        }

        static void stop_conflicting_threads(vex::thread*& current_thread) {
            if (&current_thread != (vex::thread**)&button_threads.R1) {
                stop_r1_thread();
            }
            if (&current_thread != (vex::thread**)&button_threads.R2) {
                stop_r2_thread();
            }
            if (&current_thread != (vex::thread**)&button_threads.L2) {
                stop_l2_thread();
            }
        }
    
        void ButtonL1(){
            bot::pistons::back_arm_piston.set(false);
            //bot::motors::upper.spin(vex::forward, 100, vex::percent);
        }
        void ButtonL1_released(){
            bot::pistons::back_arm_piston.set(true);
            //bot::motors::upper.stop();
            //bot::motors::mid.stop();
        }
        void ButtonL2(){
            bot::pistons::front_arm_piston.set(false);
            /*stop_conflicting_threads(button_threads.L2);
            cleanup_thread(button_threads.L2, bot::intake_methods::score_upper_running);
            if (button_threads.L2 == nullptr) {
                bot::intake_methods::score_upper_active = true;
                button_threads.L2 = new vex::thread(bot::intake_methods::score_upper);
            }
            bot::pistons::hood_piston.set(true);*/
            //bot::motors::upper.spin(vex::reverse, 100, vex::percent);
            //bot::motors::mid.spin(vex::reverse, 50, vex::percent);
        }
        void ButtonL2_released(){
            bot::pistons::front_arm_piston.set(true);
            /*stop_l2_thread();
            bot::pistons::hood_piston.set(false);*/
            //bot::motors::upper.stop();
            //bot::motors::mid.stop();
        }
        void ButtonR1(){
            stop_conflicting_threads(button_threads.R1);
            cleanup_thread(button_threads.R1, bot::intake_methods::intake_running);
            if (button_threads.R1 == nullptr) {
                bot::intake_methods::intake_active = true;
                button_threads.R1 = new vex::thread(bot::intake_methods::intake);
            }
        }
        void ButtonR1_released(){
            stop_r1_thread();
        }
        void ButtonR2(){
            stop_conflicting_threads(button_threads.R2);
            if (bot::upper_roller_direction) {
                bot::pistons::_match_load_piston.set(false);
                bot::pistons::_hood_piston.set(true);
            } else {
                bot::pistons::mid_piston.set(true);
            }
            cleanup_thread(button_threads.R2, bot::intake_methods::score_upper_running);
            if (button_threads.R2 == nullptr) {
                bot::intake_methods::score_upper_active = true;
                button_threads.R2 = new vex::thread(bot::intake_methods::score_upper);
            }
        }
        void ButtonR2_released(){
            bot::pistons::hood_piston.set(false);
            bot::pistons::mid_piston.set(false);
            stop_r2_thread();
        }
        void ButtonX(){
            bot::display_temperature();
        }
        void ButtonY(){
            //bot::intake_methods::toggle_middle_score();
            //bot::intake_methods::score_middle();
            //bot::intake_methods::intake();
            bot::intake_methods::outtake();
        }
        void ButtonY_released(){
            //bot::intake_methods::stop_scoring_middle();
            //bot::intake_methods::stop_intaking();
            bot::intake_methods::stop_outtaking();
        }
        void ButtonA(){
            //debug::print_sensor_data();
        }
        void ButtonB(){
            bot::pistons::toggle_match_load_piston();
        }
        void ButtonLeft(){
            //debug::print_location();
        }
        void ButtonRight(){

        }
        void ButtonDown(){
            //bot::pistons::toggle_arm_piston();
        }
        void ButtonUp(){
            
        }
    }
}