/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Sohum Suthar                                              */
/*    Created:      Oct 10 2021                                               */
/*    Description:  Competition Program 2360S                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <auton.h>
#include <driver.h>
#include <junk.h>
#include <vex_competition.h>

using namespace vex;
competition Competition;

void pre_auton(void) { // preauton for folding mechanisms (not in use)
  vexcodeInit();
}

void autonomous(void) { // auton
  const float P = 0.6;
  const float I = 0.4;
  const float D = 0.18;
  // driveOnPID(55, 200, 0.55, I, D);
  // move(3.9, 80);
  // move(0.35, 20);
  // Claw.setVelocity(100, percentUnits::pct);
  // Claw.spinFor(-360, rotationUnits::deg, true);
  // move(-3, 50);
  // driveOnPID(-40, 200, P, I, D);
  // 0.15 50hold
}

void usercontrol(void) {
  Controller1.ButtonY.pressed(nitroboost); // assigning all switchable modes
  Controller1.ButtonRight.pressed(snailmode);
  // Controller1.ButtonLeft.pressed(toggleonoff);
  timer Timer = timer(); // start timer for reminding the driver of time
  Timer.reset();
  float accel = 1;
  motor allmotors[] = {
      motorLB, motorLF, motorRB, motorRF, intakeB,
      intakeF, fly1,    fly2}; // check temps for overheating to fix an issue
  int i = 0;
  for (motor mymotor : allmotors) {
    if (mymotor.installed())
      continue;
    if (mymotor.temperature(fahrenheit) > 90) {
      Controller1.Screen.print("motor %d overheating!", i);
      Controller1.rumble(".....");
      break;
    }
    i++;
  }
  while (1) { // drivercontrol functions
    if (Controller1.ButtonR1.pressing()) {
      fly1.spin(directionType::fwd, 100, velocityUnits::pct);
      fly2.spin(directionType::fwd, 100, velocityUnits::pct);
    } 
    else if (Controller1.ButtonR2.pressing()) {
      indexer.set(true);
      wait(150, msec);
      indexer.set(false);
      wait(150, msec);
    } 
    else if (Controller1.ButtonL2.pressing()) {
      intakeB.spin(directionType::fwd, 100, velocityUnits::pct);
      intakeF.spin(directionType::fwd, 100, velocityUnits::pct);
    } 
    else if (Controller1.ButtonL1.pressing()) {
      intakeB.spin(directionType::fwd, 100, velocityUnits::pct);
    } 
    else if (Controller1.ButtonX.pressing()) {
      stringShooter.set(true);
      // accel -= 0.1;
    } 
    /*else if (Controller1.ButtonB.pressing()) {
    } */
    else {
      accel = 1;
      intakeF.stop(hold);
      intakeB.stop(hold);
      fly1.stop();
      fly2.stop();
    }

    // Drivetrain code for joysticks
    int turn = (Controller1.Axis3.position(vex::percent) * maxSpeedPct);
    int sideways = (Controller1.Axis4.position(vex::percent) * maxSpeedPct);
    int forward = -(Controller1.Axis1.position(vex::percent) * maxSpeedPct);

    motorRF.spin(vex::forward, forward - sideways + turn, vex::percent);
    motorLF.spin(vex::forward, forward - sideways - turn, vex::percent);
    motorRB.spin(vex::forward, forward + sideways + turn, vex::percent);
    motorLB.spin(vex::forward, forward + sideways - turn, vex::percent);
    /*
    motorLB.spin(directionType::fwd,Controller1.Axis3.position(percentUnits::pct)
    * maxSpeedPct,velocityUnits::pct);
    motorRB.spin(directionType::fwd,Controller1.Axis2.position(percentUnits::pct)
    * maxSpeedPct,velocityUnits::pct);
    motorLF.spin(directionType::fwd,Controller1.Axis3.position(percentUnits::pct)
    * maxSpeedPct,velocityUnits::pct);
    motorRF.spin(directionType::fwd,Controller1.Axis2.position(percentUnits::pct)
    * maxSpeedPct, velocityUnits::pct);
    */

    // Auxillerary systems for reminding the driver with vibrations
    if (Brain.Battery.capacity() < 15) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("CHARGE BATTERY:");
      Controller1.Screen.newLine();
      Controller1.Screen.print(Brain.Battery.capacity());
      Controller1.Screen.print("%");
      Controller1.rumble("---");
    }
    int timeRemaining = 105 - (int)Timer.value(); // 1 min 45 sec for the match

    if ((timeRemaining <= 5 && timeRemaining >= 0) ||
        timeRemaining == 10) { // vibrate the controller
      Controller1.rumble("-");
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("%d sec left", timeRemaining);
    }

    task::sleep(20); // Sleep the task for a short amount of time to prevent
                     // wasted resources.
  }
}
int main() {
  // set up all callbacks
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
