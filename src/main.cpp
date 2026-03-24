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
#include "bot/debug.hpp"
#include "pid_tuner.hpp"

using namespace vex;

static double max_speed = 100.0;

static void toggle_max_speed() {
  max_speed = (max_speed == 100.0) ? 80.0 : 100.0;
}

volatile bool is_arcade_drive = false;

static void toggle_drive_mode() {
  is_arcade_drive = !is_arcade_drive;
}

PIDTuner optimizer = PIDTuner(bot::drivetrains::dt._drive_pid);

static void tune_pid() {
  optimizer.reset();
  optimizer.set_learning_rate(0.01);
  PIDGains best_gains = optimizer.tune(bot::drive, 10);
  printf("---");
  printf("kp: %.6f, kd: %.6f", best_gains.kp, best_gains.kd);
  printf("best_time: %.1f", optimizer.get_best_time());
  printf("lr: %.6f", optimizer.get_learning_rate());
  printf("----");

  optimizer.set_learning_rate(0.001);
  best_gains = optimizer.tune(bot::drive, 10);
  printf("---");
  printf("kp: %.6f, kd: %.6f", best_gains.kp, best_gains.kd);
  printf("best_time: %.1f", optimizer.get_best_time());
  printf("lr: %.6f", optimizer.get_learning_rate());
  printf("----");

  optimizer.set_learning_rate(0.0001);
  best_gains = optimizer.tune(bot::drive, 10);
  printf("---");
  printf("kp: %.6f, kd: %.6f", best_gains.kp, best_gains.kd);
  printf("best_time: %.1f", optimizer.get_best_time());
  printf("lr: %.6f", optimizer.get_learning_rate());
  printf("----");
  return;
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

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  bot::sensors::imu.calibrate();
  while (bot::sensors::imu.isCalibrating()) {
    vex::task::sleep(10);
  }
  return;
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
  bot::autons::left_6_3();
  return;
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
  bot::Controller1.ButtonRight.pressed(toggle_drive_mode);
  //bot::Controller1.ButtonRight.pressed(bot::buttons::ButtonRight);
  bot::Controller1.ButtonDown.pressed(toggle_max_speed);
  //bot::Controller1.ButtonDown.pressed(bot::buttons::ButtonDown);
  bot::Controller1.ButtonUp.pressed(bot::buttons::ButtonUp);
  
  // variables for driver control
  double leftY, leftX, rightY, rightX;
  double left_joystick, right_joystick;
  double left, right;
  double fwd, turn;
  //double A = 0.012;
  //double K = 8;

  while (1) {
    leftY = bot::Controller1.Axis3.position();
    leftX = bot::Controller1.Axis4.position();
    rightY = bot::Controller1.Axis2.position();
    rightX = bot::Controller1.Axis1.position();

    // calculate joystick magnitudes
    left_joystick = sqrt(leftY * leftY + leftX * leftX);
    right_joystick = sqrt(rightY * rightY + rightX * rightX);

    left = left_joystick;
    right = right_joystick;

    fwd = left_joystick;
    turn = right_joystick;

    //left_joystick = A * (left_joystick - K) * (left_joystick - K);
    //right_joystick = A * (right_joystick - K) * (right_joystick - K);


    //left_joystick = A * std::pow(B, left_joystick) + C;
    //right_joystick = A * std::pow(B, right_joystick) + C;

    // deadzone adjustment
    if (fabs(left) < CONTROLLER_DEADZONE) left = 0.0;
    if (fabs(right) < CONTROLLER_DEADZONE) right = 0.0;
    if (fabs(fwd) < CONTROLLER_DEADZONE) fwd = 0.0;
    if (fabs(turn) < CONTROLLER_DEADZONE) turn = 0.0;
     
    left = (leftY < 0) ? -left_joystick : left_joystick;
    right = (rightY < 0) ? -right_joystick : right_joystick;

    fwd = (leftY < 0) ? -fwd : fwd;
    turn = (rightX < 0) ? -turn : turn;

    left = math::clamp(left, -max_speed, max_speed);
    right = math::clamp(right, -max_speed, max_speed);

    fwd = math::clamp(fwd, -max_speed, max_speed);
    turn = math::clamp(turn, -max_speed, max_speed);

    if (!is_arcade_drive) bot::drivetrains::dt.tank_drive(left, right);
    else bot::drivetrains::dt.arcade_drive(fwd, turn);

    vex::task::sleep(20); // Sleep the task for a short amount of time to
                          // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  pre_auton();

  // Set up callbacks for autonomous and driver control periods.
  bot::Competition.autonomous(autonomous);
  bot::Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
