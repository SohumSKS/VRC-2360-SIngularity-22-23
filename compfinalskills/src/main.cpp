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

  // wait(10000, timeUnits::msec);
  // strafeOnPID(30, 100, P, I, D);
  driveOnPID(5, 200, P, I, D);
  intakeB.spinFor(directionType::fwd, 0.8, rotationUnits::rev, true);
  driveOnPID(-30, 200, P, I, D);
  turnOnPID(90, 200);
  driveOnPID(30, 200, P, I, D);
  intakeB.spinFor(directionType::fwd, .8, rotationUnits::rev, true);
  driveOnPID(-15, 200, P, I, D);
  turnOnPID(200, 200);
  stringShooter2.set(true);
  turnOnPID(250, 200);
  stringShooter1.set(true);
  /*fly(true);
  driveOnPID(41, 200, P, I, D);
  turnOnPID(-35, 200);
  shoot(3, true);
  fly(false); */
}

void usercontrol(void) {
  stringShooter1.set(false);
  stringShooter2.set(false);

  // Controller1.ButtonUp.pressed(rightAngle);
  Controller1.ButtonA.pressed(nitroboost); // assigning all switchable modes
  Controller1.ButtonY.pressed(snailmode);

  // Controller1.ButtonR1.pressed(toggleFly);

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
  bool intakeState = false;
  while (1) { // drivercontrol functions
    if (Controller1.ButtonR2.pressing()) {
      indexer.set(true);
    } else if (Controller1.ButtonR1.pressing()) {
      fly1.setVelocity(42, percentUnits::pct);
      fly2.setVelocity(42, percentUnits::pct);
      fly1.spinFor(directionType::fwd, 10000, rotationUnits::rev, false);
      fly2.spinFor(directionType::fwd, 10000, rotationUnits::rev, false);
      intakeState = true;
    } else if (Controller1.ButtonL1.pressing()) {
      intakeB.spin(directionType::fwd, 100, velocityUnits::pct);
    } else if (Controller1.ButtonL2.pressing()) {
      intakeB.spin(directionType::fwd, 100, velocityUnits::pct);
      intakeF.spin(directionType::fwd, 100, velocityUnits::pct);
    } else if (Controller1.ButtonB.pressing()) {
      fly1.stop();
      fly2.stop();
    } else if (Controller1.ButtonX.pressing() &&
               Controller1.ButtonUp.pressing()) {
      stringShooter1.set(true);
      motorRF.spinTo(0.4, rotationUnits::rev, false);
      motorRB.spinTo(0.4, rotationUnits::rev, false);
      motorLF.spinTo(0.4, rotationUnits::rev, false);
      motorLB.spinTo(0.4, rotationUnits::rev, true);
      stringShooter2.set(true);

    } else if (Controller1.ButtonDown.pressing()) {
      intakeF.spin(directionType::rev, 100, percentUnits::pct);
    } else {
      accel = 1;
      if (intakeState == true) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(0, 0);
        Controller1.Screen.print(fly1.velocity(rpm) * 11.6666); // 84 12 60 36
      }
      indexer.set(false);
      intakeF.stop();
      intakeB.stop();
      intakeState = false;
    }

    // Drivetrain code for joysticks
    int turn = (output(Controller1.Axis3.position(vex::percent)) * maxSpeedPct);
    int sideways =
        (output(Controller1.Axis4.position(vex::percent)) * maxSpeedPct);
    int forward =
        -(output(Controller1.Axis1.position(vex::percent)) * maxSpeedPct);

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
      Controller1.Screen.print("Battery low:");
      Controller1.Screen.newLine();
      Controller1.Screen.print(Brain.Battery.capacity());
      Controller1.Screen.print("%");
      Controller1.rumble("-");
      wait(2500, timeUnits::msec);
    }
    int timeRemaining = 105 - (int)Timer.value(); // 1 min 45 sec for the match

    if ((timeRemaining <= 5 && timeRemaining > 0) ||
        timeRemaining == 10) { // vibrate the controller
      Controller1.rumble(".");
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
