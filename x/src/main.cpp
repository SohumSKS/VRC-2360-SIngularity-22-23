/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

float maxSpeedPct = 0.8;
bool toggle = false;
void nitroboost() { // Switchable mode
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(0, 0);
  if (maxSpeedPct == 1) {
    maxSpeedPct = 0.5;
    Controller1.rumble("..");
    Controller1.Screen.print("SLOW");
  } else {
    maxSpeedPct = 1;
    Controller1.rumble("...");
    Controller1.Screen.print("MAX SPEED");
  }

}



void usercontrol(void) {
  Controller1.ButtonA.pressed(nitroboost); //assigning all switchable modes
  // User control code here, inside the loop
  while (1) {
    left1.spin(vex::directionType::fwd, Controller1.Axis3.position() * maxSpeedPct,
               vex::velocityUnits::pct);
    left2.spin(vex::directionType::rev, Controller1.Axis3.position() * maxSpeedPct,
               vex::velocityUnits::pct);
    left3.spin(vex::directionType::fwd, Controller1.Axis3.position() * maxSpeedPct,
               vex::velocityUnits::pct);
    right1.spin(vex::directionType::rev, Controller1.Axis2.position() * maxSpeedPct,
                vex::velocityUnits::pct);
    right2.spin(vex::directionType::fwd, Controller1.Axis2.position() * maxSpeedPct,
                vex::velocityUnits::pct);
    right3.spin(vex::directionType::rev, Controller1.Axis2.position() * maxSpeedPct,
                vex::velocityUnits::pct);

    if (Controller1.ButtonR1.pressing()) {
      lift1.spin(vex::directionType::fwd, 100 * 0.75, vex::velocityUnits::pct);
      lift2.spin(vex::directionType::rev, 100 * 0.75, vex::velocityUnits::pct);
    } else if (Controller1.ButtonR2.pressing()) {
      lift1.spin(vex::directionType::rev, 100 * 0.75, vex::velocityUnits::pct);
      lift2.spin(vex::directionType::fwd, 100 * 0.75, vex::velocityUnits::pct);
    } else {
      lift1.stop(hold);
      lift2.stop(hold);
    }
    if (Controller1.ButtonL1.pressing()) {
      claw.set(false);
    } else if (Controller1.ButtonL2.pressing()) {
      claw.set(true);
    }
    if (Controller1.ButtonRight.pressing()) {
      mob.set(false);
    } else if (Controller1.ButtonY.pressing()) {
      mob.set(true);
      }
    wait(20, msec); // Sleep the task for a short amount of time to  
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
