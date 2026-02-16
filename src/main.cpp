/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       johnw                                                     */
/*    Created:      1/18/2026, 3:28:51 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "bot/bot.hpp"
#include "buttons.hpp"
#include "auton.hpp"

using namespace vex;

void auton_selector() {

  if (bot::auton_list.empty()) {
    return;
  }

  bool redraw = true;
  const int listSize = static_cast<int>(bot::auton_list.size());
  const int maxTopIndex = (listSize > 3) ? (listSize - 3) : 0;

  while (bot::currentState != bot::READY) {

    if (bot::currentState == bot::SELECTING) {

      if (redraw) {
        bot::draw_list();
        redraw = false;
      }

      // Scroll Down
      if (bot::Controller1.ButtonDown.pressing()) {
        bot::selectedIndex++;
        if (bot::selectedIndex >= listSize)
          bot::selectedIndex = listSize - 1;

        if (bot::selectedIndex > bot::topIndex + 2)
          bot::topIndex++;
        if (bot::topIndex > maxTopIndex)
          bot::topIndex = maxTopIndex;

        redraw = true;
        wait(250, msec);
      }

      // Scroll Up
      if (bot::Controller1.ButtonUp.pressing()) {
        bot::selectedIndex--;
        if (bot::selectedIndex < 0)
          bot::selectedIndex = 0;

        if (bot::selectedIndex < bot::topIndex)
          bot::topIndex--;
        if (bot::topIndex < 0)
          bot::topIndex = 0;

        redraw = true;
        wait(250, msec);
      }

      // Select
      if (bot::Controller1.ButtonA.pressing()) {
        bot::currentState = bot::CONFIRMING;
        redraw = true;
        while (bot::Controller1.ButtonA.pressing()) { wait(20, msec); }
        wait(100, msec);
      }
    }

    else if (bot::currentState == bot::CONFIRMING) {

      if (redraw) {
        bot::draw_confirm();
        redraw = false;
      }

      if (bot::Controller1.ButtonA.pressing()) {
        bot::confirmedAuton = bot::selectedIndex;
        bot::currentState = bot::READY;
        wait(300, msec);
      }
      else if (bot::Controller1.ButtonB.pressing()) {
        bot::currentState = bot::SELECTING;
        redraw = true;
        while (bot::Controller1.ButtonB.pressing()) { wait(20, msec); }
        wait(100, msec);
      }
    }

    wait(20, msec);
  }

  bot::draw_ready();
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

// Setup run before auton (calibrate IMU, etc.). Used by pre_auton and by the auton tester in usercontrol.
static void pre_auton_setup(void) {
  bot::sensors::imu.calibrate();
  vex::task::sleep(500);
}

void pre_auton(void) {
  pre_auton_setup();
  auton_selector();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  if (bot::confirmedAuton < 0 || bot::confirmedAuton >= (int)bot::auton_list.size()) {
    return;
  }

  bot::Controller1.Screen.clearScreen();
  double start_time = bot::Brain.Timer.time(vex::msec);
  bot::Controller1.Screen.setCursor(1, 1);
  bot::Controller1.Screen.print("Auton: %s", bot::auton_list[bot::confirmedAuton].c_str());

  switch (bot::confirmedAuton) {
    case 0:  bot::autons::left_7(); break;
    case 1:  bot::autons::left_4(); break;
    case 2:  bot::autons::left_4_3(); break;
    case 3:  bot::autons::left_6_3(); break;
    case 4:  bot::autons::left_6(); break;
    case 5:  bot::autons::right_7(); break;
    case 6:  bot::autons::right_6(); break;
    case 7:  bot::autons::right_4(); break;
    case 8:  bot::autons::right_4_3(); break;
    case 9:  bot::autons::sawp(); break;
    case 10: bot::autons::counter_sawp(); break;
    case 11: bot::autons::skills(); break;
    case 12: bot::autons::test(); break;
    default: break;
  }

  double end_time = bot::Brain.Timer.time(vex::msec);
  double time_taken_ms = end_time - start_time;
  bot::Controller1.Screen.setCursor(2, 1);
  bot::Controller1.Screen.print("Time: %.2f s", time_taken_ms / 1000.0);
  bot::Controller1.Screen.setCursor(3, 1);
  bot::Controller1.Screen.print("(%.0f ms)", time_taken_ms);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // Autonomous tester: select auton, run pre_auton setup, then run auton and show time
  auton_selector();
  pre_auton_setup();
  autonomous();

  bot::drivetrains::dt.coast();
  bot::motors::left_dt.stop();
  bot::motors::right_dt.stop();
  bot::motors::intake.stop();

  //bot::sensors::imu.calibrate();
  //vex::task::sleep(500);
  
  // Debug: Print IMU heading after calibration
  //printf("IMU heading after calibration: %.2f\n", bot::sensors::imu.get_heading());
  
  //bot::mcl::location.reset(300, -1200, 0);
  //bot::mcl::location.start();

  //bot::pistons::toggle_arm_piston();

  bot::Controller1.ButtonL1.pressed(bot::buttons::ButtonL1);
  bot::Controller1.ButtonL1.released(bot::buttons::ButtonL1_released);
  bot::Controller1.ButtonL2.pressed(bot::buttons::ButtonL2);
  bot::Controller1.ButtonL2.released(bot::buttons::ButtonL2_released);
  bot::Controller1.ButtonR1.pressed(bot::buttons::ButtonR1);
  bot::Controller1.ButtonR1.released(bot::buttons::ButtonR1_released);
  bot::Controller1.ButtonR2.pressed(bot::buttons::ButtonR2);
  bot::Controller1.ButtonR2.released(bot::buttons::ButtonR2_released);

  bot::Controller1.ButtonA.pressed(bot::buttons::ButtonA);
  bot::Controller1.ButtonB.pressed(bot::buttons::ButtonB);
  bot::Controller1.ButtonX.pressed(bot::buttons::ButtonX);
  bot::Controller1.ButtonY.pressed(bot::buttons::ButtonY);
  bot::Controller1.ButtonY.released(bot::buttons::ButtonY_released);

  bot::Controller1.ButtonLeft.pressed(bot::buttons::ButtonLeft);
  bot::Controller1.ButtonRight.pressed(bot::buttons::ButtonRight);
  bot::Controller1.ButtonDown.pressed(bot::buttons::ButtonDown);
  bot::Controller1.ButtonUp.pressed(bot::buttons::ButtonUp);
  
  // variables for driver control
  double leftY, leftX, rightY, rightX;
  double left_joystick, right_joystick;
  double left, right;

  while (1) {
    leftY = bot::Controller1.Axis3.position();
    leftX = bot::Controller1.Axis4.position();
    rightY = bot::Controller1.Axis2.position();
    rightX = bot::Controller1.Axis1.position();

    // calculate joystick magnitudes
    left_joystick = sqrt(leftY * leftY + leftX * leftX);
    right_joystick = sqrt(rightY * rightY + rightX * rightX);


    //left_joystick = A * std::pow(B, left_joystick) + C;
    //right_joystick = A * std::pow(B, right_joystick) + C;

    // deadzone adjustment
    if (fabs(left_joystick) < CONTROLLER_DEADZONE) left_joystick = 0.0;
    if (fabs(right_joystick) < CONTROLLER_DEADZONE) right_joystick = 0.0;
     
    left = (leftY < 0) ? -left_joystick : left_joystick;
    right = (rightY < 0) ? -right_joystick : right_joystick;

    left = math::clamp(left, -100, 100);
    right = math::clamp(right, -100, 100);

    bot::drivetrains::dt.tank_drive(left, right);

    vex::task::sleep(20); // Sleep the task for a short amount of time to
                          // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  bot::Competition.autonomous(autonomous);
  bot::Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
