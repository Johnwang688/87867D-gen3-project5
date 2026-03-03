#include "auton.hpp"

namespace bot {

    namespace drivetrains {
        bot::Drivetrain dt = bot::Drivetrain(bot::motors::left_dt, bot::motors::right_dt, bot::sensors::imu);
    }

    namespace mcl {
        bot::Location location;
    }

    namespace autons {
        using namespace bot::drivetrains;
        void left_7() {
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive(650, 1500, 100, -35);
            bot::pistons::match_load_piston.set(true);
            dt.drive(320, 1000, 50, -150);
            dt.drive(600, 1500, 70, -135);
            dt.drive(1000, 1400, 50, 180);
            dt.drive(-200, 800, 30, 180);
            dt.drive(-500, 1000, 60, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-200, 500, 40, 180);
            dt.brake();
            vex::task::sleep(1200);
            dt.coast();
            dt.drive(25, 400, 30, 180);
            dt.drive(170, 1500, 50, -90);
            bot::pistons::arm_piston.set(false);
            dt.turn_to_heading(180, 500, 100);
            dt.drive(-500, 1500, 60, 180);
            dt.brake();
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
            return;
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
            dt.drive(200, 500, 20, 180);
            dt.drive(400, 800, 50, 180);
            dt.drive(700, 1000, 40, 180);
            dt.drive(-120, 800, 50, 180);
            dt.drive(-400, 1000, 50, -135);
            dt.drive(-800, 1500, 80, -135);
            bot::motors::lower.spin(vex::forward, 8.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            dt.brake();
            dt.drive(-200, 800, 50, -135);
            vex::task mid_heading_adjust_task = vex::task([]() -> int {
                dt.turn_to_heading(-135, 1000, 100);
                return 0;
            });
            vex::task::sleep(2000);
            dt.drive(-100, 500, 30, -135);
            bot::motors::intake.stop();
            bot::pistons::match_load_piston.set(false);
            dt.drive(250, 1000, 50, -135);
            dt.drive(500, 1000, 100, -135);
            dt.drive(300, 1000, 100, -90);
            dt.drive(350, 1000, 80, 30);
            dt.turn_to_heading(170, 800, 80);
            bot::pistons::arm_piston.set(false);
            dt.drive(-650, 1500, 60, 180);
            bot::motors::right_dt.spin(vex::forward, 30, vex::percent);
            bot::motors::left_dt.spin(vex::reverse, 20, vex::percent);
            dt.brake();
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
            vex::task::sleep(15000);
            return;

        }

        void left_6_3() {
            
        }

        void left_6() {
            
        }

        void right_6() {
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive(650, 1500, 100, 35);
            dt.drive(800, 1700, 60, 90);
            bot::pistons::match_load_piston.set(true);
            dt.drive(-400, 800, 60, 0);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-450, 800, 60, -90);
            dt.drive(-200, 800, 40, 180);
            dt.drive(-300, 500, 60, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-400, 1000, 30, 180);
            dt.drive(150, 1500, 60, -90);
            dt.turn_to_heading(180, 500, 100);
            bot::pistons::arm_piston.set(false);
            dt.drive(-500, 1000, 60, 180);
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
            bot::pistons::arm_piston.set(true);
            dt.drive(-470, 1000, 60, 180);
            bot::pistons::arm_piston.set(false);
            dt.drive(-600, 1000, 60, 180);
            dt.brake();
        }

        void right_7() {
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive(650, 1500, 100, 35);
            bot::pistons::match_load_piston.set(true);
            dt.drive(300, 1000, 50, 150);
            dt.drive(600, 1500, 70, 135);
            dt.drive(1000, 1500, 50, 180);
            dt.drive(-200, 800, 30, 180);
            dt.drive(-450, 1000, 60, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-200, 500, 40, 180);
            dt.brake();
            vex::task::sleep(500);
            dt.drive(-200, 500, 70, 180);
            dt.coast();
            vex::task::sleep(200);
            dt.drive(170, 1500, 50, -90);
            bot::pistons::arm_piston.set(false);
            dt.turn_to_heading(180, 500, 100);
            dt.drive(-500, 1500, 60, 180);
            dt.brake();
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
            return;
        }

        void right_4_3() {
            bot::pistons::arm_piston.set(false);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(true);
            dt.drive(450, 1000, 100, 0);
            dt.drive(700, 1400, 50, 90);
            dt.drive(-200, 800, 20, 90);
            dt.drive(-500, 1000, 50, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.brake();
            vex::task heading_adjust_task = vex::task([]() -> int {
                dt.turn_to_heading(90, 700, 100);
                dt.drive(-300, 700, 50, 90);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            dt.coast();
            dt.drive(200, 1000, 30, 90);
            dt.drive_for(420, 1200, 60, 90);
            dt.turn_to_heading(-135, 1000, 50);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(1000, 1500, 60, -135);
            dt.drive(350, 1000, 50, -135);
            dt.brake();
            vex::task::sleep(300);
            bot::motors::lower.spin(vex::reverse, 100, vex::percent);
            vex::task::sleep(1500);
            dt.coast();
            dt.drive(-300, 700, 60, -135);
            dt.drive(-230, 700, 60, 180);
            dt.drive(-300, 700, 60, 90);
            dt.drive(-400, 700, 50, 90);
            dt.brake();
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
            dt.drive(170, 1500, 60, -90);
            dt.turn_to_heading(180, 500, 100);
            bot::pistons::arm_piston.set(false);
            dt.drive(-500, 1000, 60, 180);
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
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(650, 1500, 100, 35);
            bot::pistons::match_load_piston.set(true);
            dt.drive(-400, 800, 100, -90);
            dt.drive(-700, 700, 100, -135);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.turn_to_heading(180, 500, 100);
            dt.drive(-400, 500, 70, 180);
            dt.drive(170, 1500, 60, -90);
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
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(150, 500, 30, 0);
            dt.drive(-300, 500, 80, 0);
            dt.drive(-600, 1500, 100, 0);
            dt.drive(-300, 800, 50, 0);
            dt.drive_for(-200, 500, 100, 0);
            bot::pistons::match_load_piston.set(true);
            dt.turn_to_heading(-90, 500, 100);
            dt.drive(500, 800, 50, -90);
            dt.drive(-200, 700, 35, -90);
            dt.drive(-470, 1000, 60, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.brake();
            vex::task::sleep(700);
            dt.drive(-100, 300, 40, -90);
            dt.coast();
            dt.drive(125, 700, 40, -90);
            dt.drive(530, 1500, 50, 45);
            dt.drive(400, 800, 60, 0);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(900, 2000, 100, 0);
            dt.drive(400, 1000, 70, 0);
            bot::pistons::match_load_piston.set(true);
            dt.drive_for(200, 400, 40, 0);
            vex::task mid_scoring_task = vex::task([]() -> int {
                vex::task::sleep(150);
                bot::motors::lower.spin(vex::reverse, 4.0, vex::volt);
                vex::task::sleep(500);
                bot::motors::lower.spin(vex::forward, 8.0, vex::volt);
                bot::motors::mid.spin(vex::reverse, 100, vex::percent);
                return 0;
            });
            dt.drive(-700, 1000, 50, -45);
            dt.brake();
            vex::task mid_task = vex::task([]() -> int {
                dt.turn_to_heading(-45, 1000, 50);
                return 0;
            });
            vex::task::sleep(300);
            dt.drive(-500, 500, 20, -45);
            vex::task::sleep(700);
            bot::motors::mid.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            dt.drive(200, 500, 40, -45);
            dt.drive(700, 1500, 70, -45);
            dt.drive(200, 800, 50, -45);
            bot::pistons::match_load_piston.set(true);
            dt.drive(850, 1200, 50, -90);
            double right_distance = bot::sensors::right_dist.objectDistance(vex::mm);
            double heading_correct;
            if (right_distance == 9999 || right_distance <= 0) heading_correct = 0.0;
            else heading_correct = (500 - right_distance) * 0.10;
            dt.drive(-200, 700, 35, -90 - heading_correct);
            dt.drive(-500, 1000, 60, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            vex::task::sleep(15000);
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
        }

        void counter_sawp(){
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(100, 500, 30, 0);
            dt.drive(-300, 500, 80, 0);
            dt.drive_for(-1000, 1200, 100, 0);
            bot::pistons::match_load_piston.set(true);
            dt.turn_to_heading(-90, 500, 100);
            dt.drive(500, 800, 50, -90);
            dt.drive(-700, 1000, 80, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.brake();
            vex::task::sleep(1000);
            dt.drive_for(300, 800, 60, -90);
            dt.turn_to_heading(45, 800, 60);
            dt.drive(250, 800, 80, 45);
            dt.drive(400, 800, 60, 0);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(1000, 2000, 100, 0);
            bot::pistons::match_load_piston.set(true);
            dt.drive(300, 1000, 50, -45);
            dt.drive_for(1100, 1500, 100, -45);
            dt.turn_to_heading(-90, 500, 100);
            dt.drive(-200, 500, 100, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            //dt.drive(-200, 200, 50, -90);
            vex::task::sleep(500);
            dt.brake();
            vex::task::sleep(1000);
            dt.coast();
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(500, 1000, 100, -90);
            dt.drive(400, 800, 40, -90);
            dt.drive(-100, 800, 50, -90);
            dt.drive(-300, 800, 50, -45);
            bot::motors::lower.spin(vex::forward, 8.0, vex::volt);
            dt.drive(-1100, 1500, 100, -45);
            bot::motors::lower.spin(vex::forward, 5.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            dt.drive_for(-150, 500, 50, -45);
            dt.brake();
            vex::task::sleep(1500);
            bot::motors::mid.stop();
            bot::motors::lower.stop();
            return;
            dt.drive(250, 1000, 50, -45);
            dt.drive(500, 1000, 100, -45);
            dt.drive(300, 1000, 100, 0);
            dt.drive(350, 1000, 80, 120);
            dt.turn_to_heading(-100, 800, 100);
            bot::pistons::arm_piston.set(false);
            dt.drive(-650, 1200, 60, -90);
            dt.brake();
            bot::Controller1.Screen.setCursor(2,1);
            double end_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("end time: %.1f", end_time);
            bot::Controller1.Screen.setCursor(3,1);
            bot::Controller1.Screen.print("time taken: %.1f", end_time - start_time);
            return;
        }

        void skills() {
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive(1000, 2000, 40, 0);
            dt.drive(800, 1500, 70, 0);
            dt.brake();
            dt.turn_to_heading(90, 1000, 100);
            dt.coast();
            dt.drive(-300, 800, 40, 90);
            double left_distance = bot::sensors::left_dist.objectDistance(vex::mm);
            double heading_correct;
            if (left_distance == 9999 || left_distance <= 0) heading_correct = 0.0;
            else if (left_distance < 300) heading_correct = (400 - left_distance) * 0.07;
            else heading_correct = (850 - left_distance) * 0.07;
            vex::task::sleep(50);
            dt.drive(650, 1500, 60, 100+heading_correct);
            dt.drive_for(450 - (10.0*heading_correct), 1000, 50, 100+heading_correct);
            dt.turn_to_heading(-45, 1200, 100);
            bot::motors::lower.stop();
            bot::motors::intake.spin(vex::reverse, 5.0, vex::volt);
            dt.drive(-450 - (20.0*heading_correct), 1000, 60, -45);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 11.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            dt.brake();
            vex::task mid_task = vex::task([]() -> int {
                dt.turn_to_heading(-45, 1500, 100);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::lower.spin(vex::forward, 11.0, vex::volt);
            vex::task::sleep(1000);
            bot::motors::mid.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(-100, 500, 30, -45);
            dt.drive(300, 1000, 50, -45);
            bot::pistons::match_load_piston.set(true);
            dt.drive(750, 1500, 70, -45);
            dt.drive(700, 1500, 50, -90);
            dt.drive(500, 1200, 40, -90);
            dt.turn_to_heading(-90, 500, 100);
            double right_distance = bot::sensors::right_dist.objectDistance(vex::mm);
            double heading_correct_2;
            if (right_distance == 9999 || right_distance <= 0) heading_correct_2 = 0.0;
            else heading_correct_2 = (600 - right_distance);
            dt.drive(-250 - sqrt(2.0)*heading_correct_2, 1500, 40, -45);
            dt.drive(-350, 1000, 40, -45);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-200, 1000, 40, -45);
            dt.drive(-400, 1500, 80, -45);
            dt.drive_for(-200, 500, 50, -45);
            bot::motors::lower.spin(vex::reverse, 8.0, vex::volt);
            dt.drive(-300, 500, 35, -45);
            bot::motors::lower.spin(vex::forward, 10.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 5.5, vex::volt);
            mid_task = vex::task([]() -> int {
                dt.turn_to_heading(-45, 1000, 100);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::mid.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(150, 800, 30, -45);
            dt.drive(500, 1500, 50, 85);
            dt.drive(650, 1500, 60, 80);
            double left_distance_2 = bot::sensors::left_dist.objectDistance(vex::mm);
            double dsr_distance;
            if (left_distance_2 == 9999 || left_distance_2 <= 0) dsr_distance = 350.0;
            else if (left_distance_2 < 700.0) dsr_distance = left_distance_2 - 200.0;
            else dsr_distance = left_distance_2 - 650.0;
            bot::pistons::match_load_piston.set(true);
            dt.drive(400, 1500, 50, 20);
            dt.drive(dsr_distance, 1200, 60, 20);
            dt.drive(200, 1000, 30, 90);
            dt.drive(-400, 1000, 40, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            vex::task heading_adjust_task = vex::task([]() -> int {
                dt.drive(20, 200, 25, 90);
                dt.brake();
                dt.turn_to_heading(90, 1000, 100);
                dt.drive(-100, 500, 50, 90);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(200, 800, 20, 90);
            dt.drive(450, 800, 50, 90);
            dt.drive(700, 1700, 40, 90);
            dt.drive(-300, 1000, 30, 90);
            dt.drive(-400, 1000, 70, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            heading_adjust_task = vex::task([]() -> int {
                dt.brake();
                dt.turn_to_heading(90, 1500, 100);
                dt.drive(-500, 500, 40, 90);
                return 0;
            });
            vex::task::sleep(2000);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            bot::pistons::match_load_piston.set(false);
            dt.drive(600, 800, 80, 135);
            dt.drive(500, 800, 100, 180);
            dt.turn_to_heading(180, 500, 100);
            bot::sensors::imu.setHeading(180, vex::degrees);
            dt.coast();
            dt.drive(1000, 2000, 40, 180);
            dt.drive(400, 1500, 100, 180);
            dt.drive(600, 1500, 60, -120);
            dt.drive(1500, 1200, 60, 180);
            dt.drive(-200, 800, 50, 135);
            vex::task intake_task = vex::task([]() -> int {
                vex::task::sleep(1000);
                bot::motors::intake.spin(vex::forward, 100, vex::percent);
                return 0;
            });
            dt.drive(-300, 600, 40, 90);
            dt.drive(-1000, 1000, 50, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            bot::pistons::match_load_piston.set(true);
            vex::task::sleep(500);
            /*
            heading_adjust_task = vex::task([]() -> int {
                dt.turn_to_heading(90, 500, 100);
                dt.drive(-500, 500, 40, 90);
                return 0;
            });
            vex::task::sleep(1000);
            */
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            dt.drive(200, 800, 20, 90);
            dt.drive(350, 1000, 50, 90);
            dt.drive(600, 2000, 50, 90);
            dt.drive(-370, 1000, 50, 45);
            dt.drive(-300, 1000, 50, 90);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-1700, 3000, 60, 85);
            dt.drive(-400, 1500, 60, 180);
            dt.drive(-300, 1200, 30, -90);
            intake_task = vex::task([]() -> int {
                vex::task::sleep(500);
                bot::motors::intake.spin(vex::forward, 100, vex::percent);
                return 0;
            });
            dt.drive(-600, 1000, 40, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            heading_adjust_task = vex::task([]() -> int {
                dt.drive(20, 200, 25, -90);
                dt.turn_to_heading(-90, 1000, 100);
                bot::pistons::match_load_piston.set(true);
                dt.drive(-500, 800, 40, -90);
                return 0;
            });
            vex::task::sleep(2000);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            dt.drive(200, 800, 20, -90);
            dt.drive(350, 1000, 50, -90);
            dt.drive(600, 1700, 50, -90);
            dt.drive(-300, 1000, 30, -90);
            dt.drive(-400, 1000, 70, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            dt.drive(-500, 500, 40, -90);
            heading_adjust_task = vex::task([]() -> int {
                dt.drive(20, 200, 25, -90);
                dt.turn_to_heading(-90, 1500, 100);
                bot::pistons::match_load_piston.set(false);
                return 0;
            });
            vex::task::sleep(2200);
            dt.drive(25, 400, 100, -90);
            dt.drive(300, 1000, 80, -30);
            dt.drive(300, 1000, 80, -45);
            dt.drive(500, 1000, 100, 0);
            dt.drive(800, 2000, 50, 0);
            dt.brake();
        }

        void skills_80(){
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::sensors::imu.setHeading(90, vex::degrees);
            bot::pistons::arm_piston.set(true);
            dt.drive_for(850, 1500, 60, 90);
            dt.turn_to_heading(180, 1000, 50);
            bot::pistons::match_load_piston.set(true);
            vex::task::sleep(500);
            dt.drive(1000, 2000, 50, 180);
            dt.drive(-400, 1000, 50, -135);
            dt.drive(-300, 1000, 40, 180);
            bot::pistons::match_load_piston.set(false);
            bot::motors::lower.stop();
            dt.drive(-700, 1500, 70, 180);
            dt.drive(-300, 1000, 60, 180);
            dt.drive(-700, 1500, 60, 180);
            dt.drive(-380, 1000, 50, 90);
            dt.drive(-400, 1000, 40, 0);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-500, 800, 40, 0);
            dt.brake();
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            dt.drive(200, 800, 30, 0);
            bot::pistons::match_load_piston.set(true);
            dt.drive(1000, 2000, 50, 0);
            dt.drive(-200, 800, 30, 0);
            dt.drive(-450, 1000, 50, 0);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-500, 500, 40, 0);
            dt.brake();
            vex::task::sleep(1500);
            dt.coast();
            bot::motors::intake.stop();
            bot::pistons::match_load_piston.set(false);
            dt.drive(600, 800, 80, -45);
            dt.drive(500, 800, 100, -90);
            dt.turn_to_heading(-90, 500, 100);
            dt.brake();
            vex::task::sleep(200);
            dt.coast();
            bot::sensors::imu.setHeading(270, vex::degrees);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(1200, 2000, 40, -90);
            dt.drive(400, 1500, 60, -90);
            dt.drive(600, 1500, 60, -160);
            dt.drive(2000, 1500, 60, -90);
            dt.drive(-200, 800, 50, -45);
            dt.drive(-300, 600, 40, 0);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-500, 800, 50, 0);
            dt.drive(25, 500, 30, 0);
            dt.turn_to_heading(0, 500, 100);
            dt.drive(-500, 800, 40, 0);
            dt.brake();
            vex::task::sleep(500);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            bot::pistons::match_load_piston.set(true);
            dt.drive(200, 800, 30, 0);
            dt.drive(1000, 2000, 50, 0);
            dt.drive(-400, 1000, 50, 45);
            dt.drive(-300, 1000, 40, 0);
            bot::pistons::match_load_piston.set(false);
            bot::motors::lower.stop();
            dt.drive(-700, 1500, 70, 0);
            dt.drive(-300, 1000, 60, 0);
            dt.drive(-700, 1500, 60, 0);
            dt.drive(-380, 1000, 50, -90);
            dt.drive(-400, 1000, 40, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-500, 800, 40, 180);
            dt.brake();
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            bot::pistons::match_load_piston.set(true);
            dt.drive(200, 800, 30, 180);
            dt.drive(1000, 2000, 50, 180);
            dt.drive(-200, 800, 30, 180);
            dt.drive(-450, 1000, 50, 180);
            bot::pistons::match_load_piston.set(false);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.drive(-500, 800, 40, 180);
            dt.brake();
            vex::task::sleep(1500);
            bot::motors::intake.stop();
            dt.coast();
            dt.drive(600, 800, 80, 135);
            dt.drive(500, 800, 100, 90);
            dt.turn_to_heading(90, 500, 100);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(800, 2500, 40, 90);
            dt.drive(-150, 1000, 100, 90);
            dt.brake();


        }

        void test() {
            std::vector<PathPoint> path = {
                {500, -1000, 1},
                {600, -600, 1}
                
            };
            std::vector<PathPoint> path_2 = {
                {800, -800, 1},
                {1000, -1000, 1},
                {1200, -1200, 1},
                {1200, -1400, 1}
            };
            dt.pure_pursuit(path, 200, 30, 10000);
            dt.turn_to_heading(135, 800, 100);
            dt.pure_pursuit(path_2, 200, 30, 10000);
            return;
        }

        void no_auton() {
            dt.drive(30, 800, 30, 0);
            dt.brake();
        }
    }

}