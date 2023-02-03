#ifndef JUNK_H
#define JUNK_H
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       junk.h                                                   */
/*    Author:       Sohum Suthar                                              */
/*    Created:      Oct 10 2021                                               */
/*    Description:  Competition Program 2360S                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <vex_competition.h>
#include <auton.h>

using namespace vex;

void gyroTurnPID(double referenceHeading, double kp, double ki,double kd) {
  double lastError = 0;
  float integralSum = 0;
  timer period = timer();

  if (referenceHeading < 0) {
    while (imu.angle(rotationUnits::deg) <= referenceHeading) {
      period.reset();
      double error;
      double imuHeading = imu.angle(rotationUnits::deg);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      motorLF.setVelocity(100 * output, velocityUnits::pct);
      motorLB.setVelocity(100 * output, velocityUnits::pct);
      motorRB.setVelocity(100 * output, velocityUnits::pct);
      motorRF.setVelocity(100 * output, velocityUnits::pct);

      motorLF.spin(directionType::rev);
      motorRF.spin(directionType::fwd);
      motorLB.spin(directionType::rev);
      motorRB.spin(directionType::fwd);

      lastError = error;
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }

  } else {
    while (imu.angle(rotationUnits::deg) <= referenceHeading) {
      period.reset();
      double error;
      double imuHeading = imu.angle(rotationUnits::deg);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      motorLF.setVelocity(100 * output, velocityUnits::pct);
      motorLB.setVelocity(100 * output, velocityUnits::pct);
      motorRB.setVelocity(100 * output, velocityUnits::pct);
      motorRF.setVelocity(100 * output, velocityUnits::pct);

      motorLF.spin(directionType::fwd);
      motorRF.spin(directionType::rev);
      motorLB.spin(directionType::fwd);
      motorRB.spin(directionType::rev);

      lastError = error;
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }
  }
  driveStop(true);
}
double angleCalc(double requestedAngle){
  double revs;
  revs = (((sqrt(574.65) * M_PI) / 360) * requestedAngle) / 4 * M_PI;
  return revs;
}
void gyroTurn(double referenceHeading, int veloc) {
  double imuHeading = imu.angle(rotationUnits::deg);
  // Set speeds of both Drive motors
  motorLF.setVelocity(veloc, velocityUnits::pct);
  motorLB.setVelocity(veloc, velocityUnits::pct);
  motorRB.setVelocity(veloc, velocityUnits::pct);
  motorRF.setVelocity(veloc, velocityUnits::pct);

  // Prints the referenceHeading for debugging puroses to ensure that it is
  // going for the right degree amount

  // While loop to do the spin
  if (referenceHeading < 0) {
    while (imuHeading <= referenceHeading) {
      motorLF.spin(directionType::rev);
      motorRF.spin(directionType::fwd);
      motorLB.spin(directionType::rev);
      motorRB.spin(directionType::fwd);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }

  } else {
    while (imuHeading <= referenceHeading) {
      motorLF.spin(directionType::fwd);
      motorRF.spin(directionType::rev);
      motorLB.spin(directionType::fwd);
      motorRB.spin(directionType::rev);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }
  }

  // Stop motors after reached degree turn
  driveStop(true);
  imu.setHeading(0, rotationUnits::deg);
}
void move(double revs, double power) {
  motorLB.setVelocity(power, velocityUnits::pct);
  motorLF.setVelocity(power, velocityUnits::pct);
  motorRB.setVelocity(power, velocityUnits::pct);
  motorRF.setVelocity(power, velocityUnits::pct);

  motorLB.spinFor(revs, rotationUnits::rev, false);
  motorLF.spinFor(revs, rotationUnits::rev, false);
  motorRB.spinFor(revs, rotationUnits::rev, false);
  motorRF.spinFor(revs, rotationUnits::rev, false);
}
void moveIn(double inches, int poder) {
  double revs = inches / (4.000 * M_PI);
  motorLB.setVelocity(poder, velocityUnits::pct);
  motorLF.setVelocity(poder, velocityUnits::pct);
  motorRB.setVelocity(poder, velocityUnits::pct);
  motorRF.setVelocity(poder, velocityUnits::pct);

  motorLB.spinFor(revs, rotationUnits::rev, false);
  motorLF.spinFor(revs, rotationUnits::rev, false);
  motorRB.spinFor(revs, rotationUnits::rev, false);
  motorRF.spinFor(revs, rotationUnits::rev, true);
}
void balance(double power){
  while(imu.pitch(rotationUnits::deg) > 5 || imu.pitch(rotationUnits::deg) < -5){
    motorLB.setVelocity(power, velocityUnits::pct);
    motorLF.setVelocity(power, velocityUnits::pct);
    motorRB.setVelocity(power, velocityUnits::pct);
    motorRF.setVelocity(power, velocityUnits::pct);
    motorLF.spin(directionType::fwd);
    motorRF.spin(directionType::fwd);
    motorLB.spin(directionType::fwd);
    motorRB.spin(directionType::fwd);
  }
  driveStop(true);
}
void turnNoIMU(double referenceHeading, double power, double kp, double ki, double kd){
  double lastError = 0;
  float integralSum = 0;
  timer period = timer();
  motorLF.resetRotation();
  if (referenceHeading < 0) {
    while ((double)motorLB.rotation(rotationUnits::rev) <= (double)angleCalc(referenceHeading)) {
      period.reset();
      double error;
      double imuHeading = motorLB.rotation(rotationUnits::rev);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      motorLF.setVelocity(100 * output, velocityUnits::pct);
      motorLB.setVelocity(100 * output, velocityUnits::pct);
      motorRB.setVelocity(100 * output, velocityUnits::pct);
      motorRF.setVelocity(100 * output, velocityUnits::pct);

      motorLF.spin(directionType::rev);
      motorRF.spin(directionType::fwd);
      motorLB.spin(directionType::rev);
      motorRB.spin(directionType::fwd);

      lastError = error;
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }
  }
   else {
    while ((double)motorLB.rotation(rotationUnits::rev) >= (double)angleCalc(referenceHeading)){
      period.reset();
      double error;
      double imuHeading = imu.angle(rotationUnits::deg);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      motorLF.setVelocity(100 * output, velocityUnits::pct);
      motorLB.setVelocity(100 * output, velocityUnits::pct);
      motorRB.setVelocity(100 * output, velocityUnits::pct);
      motorRF.setVelocity(100 * output, velocityUnits::pct);

      motorLF.spin(directionType::fwd);
      motorRF.spin(directionType::rev);
      motorLB.spin(directionType::fwd);
      motorRB.spin(directionType::rev);

      lastError = error;
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }
  }
  driveStop(true);
}

void turnAngle(double angle, double poder){
  if (angle > 0){
    motorLF.rotateTo((double)angleCalc(angle), rotationUnits::rev, poder, velocityUnits::pct, false);
    motorRF.rotateTo((double)angleCalc(angle) , rotationUnits::rev, -poder, velocityUnits::pct, false);
    motorLB.rotateTo((double)angleCalc(angle), rotationUnits::rev, poder, velocityUnits::pct, false);
    motorRB.rotateTo((double)angleCalc(angle) , rotationUnits::rev, -poder, velocityUnits::pct, true);
  }
  else if(angle < 0){
    motorLF.rotateTo((double)angleCalc(angle), rotationUnits::rev, -poder, velocityUnits::pct, false);
    motorRF.rotateTo((double)angleCalc(angle) , rotationUnits::rev, poder, velocityUnits::pct, false);
    motorLB.rotateTo((double)angleCalc(angle), rotationUnits::rev, -poder, velocityUnits::pct, false);
    motorRB.rotateTo((double)angleCalc(angle) , rotationUnits::rev, poder, velocityUnits::pct, true);
  }
  driveStop(true);
}

void moveInches(double inches, double poder){
  motorLF.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, false);
  motorRF.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, false);
  motorLB.rotateTo((inches / (4 * M_PI)), rotationUnits::rev, poder, velocityUnits::pct, false);
  motorRB.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, true);
}
#endif