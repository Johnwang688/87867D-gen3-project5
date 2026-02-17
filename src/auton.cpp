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
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            bot::pistons::arm_piston.set(true);
            dt.drive(650, 1500, 100, -35);
            bot::pistons::match_load_piston.set(true);
            dt.drive(350, 1000, 50, -150);
            dt.drive(600, 1500, 70, -135);
            dt.drive(1000, 1500, 50, 180);
            dt.drive(-100, 1000, 40, 180);
            dt.drive(-550, 1000, 70, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-200, 500, 40, 180);
            dt.brake();
            vex::task::sleep(1000);
            dt.coast();
            dt.drive(150, 1500, 50, -90);
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
            dt.drive(550, 1000, 60, 180);
            dt.drive(450, 800, 40, 180);
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
            dt.drive(350, 1000, 50, 150);
            dt.drive(600, 1500, 70, 135);
            dt.drive(1000, 1500, 50, 180);
            dt.drive(-100, 1000, 40, 180);
            dt.drive(-550, 1000, 70, 180);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-200, 500, 40, 180);
            dt.brake();
            vex::task::sleep(1000);
            dt.coast();
            dt.drive(150, 1500, 50, -90);
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
            // TODO: implement Right 4+3 auton
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
            bot::pistons::arm_piston.set(true);
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
            double start_time = bot::Brain.Timer.time(vex::msec);
            bot::Controller1.Screen.clearScreen();
            bot::Controller1.Screen.setCursor(1,1);
            bot::Controller1.Screen.print("start time: %.1f", start_time);
            bot::pistons::arm_piston.set(true);
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(100, 500, 30, 0);
            dt.drive(-300, 500, 80, 0);
            dt.drive_for(-1000, 1300, 100, 0);
            bot::pistons::match_load_piston.set(true);
            dt.turn_to_heading(-90, 500, 100);
            dt.drive(500, 1000, 50, -90);
            dt.drive(-650, 1000, 80, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            bot::pistons::match_load_piston.set(false);
            dt.brake();
            vex::task::sleep(1000);
            dt.coast();
            dt.drive_for(300, 700, 60, -90);
            dt.turn_to_heading(45, 800, 50);
            dt.drive(300, 800, 80, 45);
            dt.drive(400, 800, 60, 0);
            bot::motors::intake.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(1100, 2000, 100, 0);
            dt.drive_for(580, 1000, 80, 0);
            dt.drive(-500, 1000, 50, -45);
            dt.brake();
            bot::motors::lower.spin(vex::forward, 8.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            vex::task mid_task = vex::task([]() -> int {
                dt.turn_to_heading(-45, 1000, 50);
                return 0;
            });
            vex::task::sleep(1500);
            bot::motors::mid.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.coast();
            dt.drive(1020, 1500, 80, -45);
            bot::pistons::match_load_piston.set(true);
            dt.drive(850, 1500, 50, -90);
            dt.drive(-650, 1000, 80, -90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            vex::task::sleep(1000);
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
            dt.drive(800, 1000, 100, 0);
            dt.drive(200, 800, 20, 0);
            dt.turn_to_heading(90, 1000, 100);
            dt.drive(-300, 800, 40, 90);
            dt.drive(825, 1500, 60, 102);
            dt.turn_to_heading(-45, 1000, 100);
            bot::motors::lower.stop();
            dt.drive(-400, 1000, 60, -45);
            bot::motors::lower.spin(vex::forward, 8.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            dt.brake();
            vex::task mid_task = vex::task([]() -> int {
                dt.turn_to_heading(-45, 1500, 100);
                return 0;
            });
            vex::task::sleep(2000);
            bot::motors::lower.spin(vex::forward, 11.0, vex::volt);
            vex::task::sleep(2000);
            bot::motors::mid.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(300, 1000, 50, -45);
            bot::pistons::match_load_piston.set(true);
            dt.drive(725, 1500, 70, -45);
            dt.drive(700, 1500, 50, -90);
            dt.drive(500, 1000, 40, -90);
            dt.turn_to_heading(-90, 500, 100);
            dt.drive_for(-150, 1000, 50, -90);
            dt.turn_to_heading(-43, 1000, 100);
            bot::pistons::match_load_piston.set(false);
            dt.drive_for(-1500, 2000, 80, -45);
            bot::motors::lower.spin(vex::forward, 8.0, vex::volt);
            bot::motors::mid.spin(vex::reverse, 100, vex::percent);
            vex::task::sleep(1500);
            bot::motors::mid.stop();
            bot::motors::lower.spin(vex::forward, 100, vex::percent);
            dt.drive(400, 1500, 50, 80);
            dt.drive(600, 1500, 60, 80);
            bot::pistons::match_load_piston.set(true);
            dt.drive(750, 1500, 50, 20);
            dt.brake();
            dt.turn_to_heading(90, 1000, 100);
            dt.drive(-300, 1000, 80, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            dt.brake();
            vex::task::sleep(2000);
            return;
            dt.drive(-300, 1000, 50, -135);
            dt.drive(-300, 1000, 50, -90);
            bot::pistons::match_load_piston.set(false);
            dt.drive(-1400, 2500, 80, -90);
            dt.drive(-300, 1500, 50, 0);
            dt.drive(-300, 1000, 30, 90);
            bot::motors::intake.spin(vex::forward, 100, vex::percent);
            vex::task::sleep(2000);
            bot::motors::intake.stop();
            return;
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
            bot::pistons::arm_piston.set(true);
            dt.drive(1000, 2000, 40, 0);
            dt.drive(800, 1000, 100, 0);
            dt.drive(200, 800, 20, 0);
            return;
        }
    }

    std::vector<std::string> auton_list = {
        "Left 7",
        "Left 4",
        "Left 4+3",
        "Left 6+3",
        "Left 6",
        "Right 7",
        "Right 6",
        "Right 4",
        "Right 4+3",
        "Sawp",
        "Counter Sawp",
        "Skills",
        "Test auto"
    };

    int selectedIndex = 0;
    int topIndex = 0;
    int confirmedAuton = -1;
    SelectorState currentState = SELECTING;

    SelectorState get_current_state() {
        return currentState;
    }

    void draw_list() {
        if (auton_list.empty()) return;

        const int size = static_cast<int>(auton_list.size());
        const int maxTop = (size > 3) ? (size - 3) : 0;
        const int safeTop = std::max(0, std::min(topIndex, maxTop));

        bot::Controller1.Screen.clearScreen();
        for (int i = 0; i < 3; i++) {
            int autonIndex = safeTop + i;
            if (autonIndex >= size) break;

            bot::Controller1.Screen.setCursor(i + 1, 1);
            if (autonIndex == selectedIndex)
                bot::Controller1.Screen.print("> %s", auton_list[autonIndex].c_str());
            else
                bot::Controller1.Screen.print("  %s", auton_list[autonIndex].c_str());
        }
    }

    void draw_confirm() {
        bot::Controller1.Screen.clearScreen();
        bot::Controller1.Screen.setCursor(1, 1);
        bot::Controller1.Screen.print("Select:");
        bot::Controller1.Screen.setCursor(2, 1);
        if (selectedIndex >= 0 && selectedIndex < static_cast<int>(auton_list.size()))
            bot::Controller1.Screen.print("%s", auton_list[selectedIndex].c_str());
        else
            bot::Controller1.Screen.print("(invalid)");
        bot::Controller1.Screen.setCursor(3, 1);
        bot::Controller1.Screen.print("A=Yes  B=No");
    }

    void draw_ready() {
        bot::Controller1.Screen.clearScreen();
        bot::Controller1.Screen.setCursor(1, 1);
        bot::Controller1.Screen.print("Auton selected:");
        bot::Controller1.Screen.setCursor(2, 1);
        if (confirmedAuton >= 0 && confirmedAuton < static_cast<int>(auton_list.size()))
            bot::Controller1.Screen.print("%s", auton_list[confirmedAuton].c_str());
        else
            bot::Controller1.Screen.print("(none)");
    }
}