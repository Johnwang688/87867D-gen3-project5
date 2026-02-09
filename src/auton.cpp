#include "auton.hpp"

namespace bot {

    namespace drivetrains {
        bot::Drivetrain dt = bot::Drivetrain(bot::motors::left_dt, bot::motors::right_dt, bot::sensors::imu);
    }

    namespace mcl {
        //bot::Location location;
    }

    namespace autons {
        using namespace bot::drivetrains;
        void left_7() {

        }

        void left_4_3() {
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive(650, 1500, 100, -35);
            bot::pistons::match_load_piston.set(true);
            dt.drive(-480, 1500, 70, 60);
            dt.drive(-900, 1200, 50, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task heading_adjust_task = vex::task([]() -> int {
                dt.turn_to_heading(180, 1000, 100);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(550, 1000, 60, 180);
            dt.drive(450, 800, 40, 180);
            dt.drive(-120, 800, 50, 180);
            dt.drive(-400, 1000, 50, -135);
            dt.drive(-800, 1500, 80, -135);
            bot::motors::lower.spin(vex::forward, 5.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            dt.brake();
            dt.drive(-200, 800, 50, -135);
            vex::task mid_heading_adjust_task = vex::task([]() -> int {
                dt.turn_to_heading(-135, 1000, 100);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            bot::pistons::match_load_piston.set(false);
            dt.drive(250, 1000, 50, -135);
            dt.drive(500, 1000, 100, -135);
            dt.drive(300, 1000, 100, -90);
            dt.drive(350, 1000, 80, 30);
            dt.turn_to_heading(170, 800, 100);
            bot::pistons::arm_piston.set(false);
            dt.drive(-650, 1200, 60, 180);
            dt.brake();
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
            return;

        }

        void left_6_3() {
            
        }

        void right_7() {

        }

        void left_4() {
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive(650, 1500, 100, -35);
            bot::pistons::match_load_piston.set(true);
            dt.drive(-400, 800, 100, 90);
            dt.drive(-700, 800, 100, -135);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.turn_to_heading(180, 500, 100);
            dt.drive(-400, 500, 70, 180);
            dt.drive(200, 800, 60, -90);
            dt.drive(200, 800, 60, 180);
            bot::pistons::arm_piston.set(false);
            dt.drive(-650, 1000, 60, 180);
            dt.brake();
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
        }

        void right_4() {
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(650, 1500, 100, 35);
            bot::pistons::match_load_piston.set(true);
            dt.drive(-400, 800, 100, -90);
            dt.drive(-700, 700, 100, -135);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-400, 1000, 70, 180);
            dt.drive(150, 1500, 60, -90);
            bot::motors::intake.stop();
            dt.turn_to_heading(-180, 500, 100);
            bot::pistons::arm_piston.set(false);
            dt.drive(-500, 1000, 60, 180);
            dt.brake();
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
        }

        void sawp() {
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(100, 500, 30, 0);
            dt.drive(-300, 500, 80, 0);
            dt.drive_for(-1000, 1500, 100, 0);
            bot::pistons::match_load_piston.set(true);
            dt.turn_to_heading(-90, 800, 100);
            dt.drive(500, 1000, 50, -90);
            dt.drive(-650, 1000, 70, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-400, 1000, 35, -90);
            dt.drive_for(300, 1000, 60, -90);
            dt.turn_to_heading(45, 800, 100);
            dt.drive(300, 800, 80, 45);
            dt.drive(400, 800, 60, 0);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(1100, 2000, 100, 0);
            dt.drive_for(550, 1000, 80, 0);
            dt.drive(-500, 1000, 50, -45);
            dt.brake();
            bot::motors::lower.spin(vex::forward, 5.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            vex::task mid_task = vex::task([]() -> int {
                dt.turn_to_heading(-45, 1000, 50);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::mid.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            dt.drive(1000, 1500, 70, -45);
            bot::pistons::match_load_piston.set(true);
            dt.drive(850, 1500, 50, -90);
            dt.drive(-700, 1000, 70, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-400, 2000, 40, -90);

        }

        void skills() {
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(500, 2000, 40, 0);
            dt.drive(800, 3000, 60, 0);
            bot::pistons::match_load_piston.set(true);
            vex::task::sleep(100);
            dt.drive(-600, 1500, 25, 0);
            bot::pistons::match_load_piston.set(false);
            dt.drive(400, 1500, 50, 90);
            dt.drive_for(600, 1500, 50, 90);
            dt.turn_to_heading(-45, 1000, 50);
            bot::motors::lower.spin(vex::reverse, 4.0, vex::volt);
            dt.drive(-400, 1000, 50, -45);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            bot::motors::lower.spin(vex::forward, 5.0, vex::volt);
            dt.drive(-400, 5000, 30, -45);
            return;
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(true);
            bot::pistons::arm_piston.set(true);
            dt.drive(450, 1000, 100, 0);
            dt.drive_for(500, 1000, 50, 90);
            dt.drive_for(700, 1000, 30, 90);
            vex::task::sleep(1000);
            dt.drive_for(-300, 1000, 60, 90);
            bot::pistons::match_load_piston.set(false);
            dt.turn_to_heading(-45, 1000, 50);
            dt.drive(500, 1000, 60, -45);
            dt.drive(600, 1200, 70, -90);
            dt.drive(1300, 2000, 70, -90);
            dt.drive(520, 1000, 50, -160);
            dt.drive(-800, 1000, 50, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(true);
            dt.drive(-200, 500, 40, -90);
            vex::task::sleep(1000);
            dt.drive(-300, 1000, 50, -90);
            vex::task::sleep(2000);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(400, 1000, 60, -90);
            dt.drive(300, 500, 60, -90);
            dt.drive_for(400, 2000, 30, -90);
            dt.drive(-500, 1000, 70, -90);
            dt.drive(-200, 500, 50, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-300, 500, 40, -90);
            vex::task::sleep(2000);
            dt.drive(-400, 800, 50, -90);
            vex::task::sleep(1000);
            dt.turn_to_heading(-90, 1000, 50);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.drive(400, 1000, 50, -135);
            dt.drive(400, 1000, 50, 180);
            dt.drive(900, 1500, 70, 180);
            dt.drive(900, 1500, 70, 180);
            bot::pistons::match_load_piston.set(true);
            dt.drive(500, 1000, 50, -90);
            dt.drive_for(600, 2000, 40, -90);
            vex::task::sleep(500);
            dt.drive_for(-250, 1000, 60, -90);
            bot::pistons::match_load_piston.set(false);
            dt.turn_to_heading(135, 1500, 50);
            dt.drive(500, 1000, 50, 135);
            dt.drive(600, 1200, 70, 90);
            dt.drive(1200, 2000, 70, 90);
            dt.drive(500, 1000, 50, 20);
            dt.drive(-800, 1000, 50, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-300, 500, 40, 90);
            bot::pistons::match_load_piston.set(true);
            vex::task::sleep(3000);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(400, 1000, 60, 90);
            dt.drive(300, 500, 60, 90);
            dt.drive_for(400, 2000, 30, 90);
            vex::task::sleep(500);
            dt.drive(-500, 1000, 70, 90);
            dt.drive(-200, 500, 50, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-300, 500, 40, 90);
            vex::task::sleep(3000);
            bot::pistons::match_load_piston.set(false);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(1000, 1500, 50, 45);
            dt.drive(400, 1000, 50, 15);
            bot::pistons::match_load_piston.set(true);
            dt.drive(1000, 1000, 30, 0);
            dt.drive_for(500, 2000, 80, 0);
        }

        void test() {
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::motors::upper.spin(vex::reverse, 11.0, vex::volt);
            vex::task piston_task = vex::task([]() -> int {
                vex::task::sleep(500);
                bot::pistons::match_load_piston.set(true);
                return 0;
            });
            dt.drive_for(700, 1000, 100, -20);
            dt.drive_for(-1000, 1500, 100, 90);
            dt.turn_to_heading(180, 1000, 100);
            dt.drive_for(750, 1000, 60, 180);
            dt.turn_to_heading(180, 500, 50);
            dt.drive_for(-800, 1500, 100, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task::sleep(2000);
            bot::motors::intake.stop();
            bot::pistons::match_load_piston.set(false);
            dt.drive_for(350, 1000, 50, 90);
            dt.turn_to_heading(180, 1000, 100);
            dt.drive_for(-600, 1500, 100, 180);
            return;
        }
    }


}