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

      MotorLF.setVelocity(100 * output, velocityUnits::pct);
      MotorLB.setVelocity(100 * output, velocityUnits::pct);
      MotorRB.setVelocity(100 * output, velocityUnits::pct);
      MotorRF.setVelocity(100 * output, velocityUnits::pct);

      MotorLF.spin(directionType::rev);
      MotorRF.spin(directionType::fwd);
      MotorLB.spin(directionType::rev);
      MotorRB.spin(directionType::fwd);

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

      MotorLF.setVelocity(100 * output, velocityUnits::pct);
      MotorLB.setVelocity(100 * output, velocityUnits::pct);
      MotorRB.setVelocity(100 * output, velocityUnits::pct);
      MotorRF.setVelocity(100 * output, velocityUnits::pct);

      MotorLF.spin(directionType::fwd);
      MotorRF.spin(directionType::rev);
      MotorLB.spin(directionType::fwd);
      MotorRB.spin(directionType::rev);

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
  MotorLF.setVelocity(veloc, velocityUnits::pct);
  MotorLB.setVelocity(veloc, velocityUnits::pct);
  MotorRB.setVelocity(veloc, velocityUnits::pct);
  MotorRF.setVelocity(veloc, velocityUnits::pct);

  // Prints the referenceHeading for debugging puroses to ensure that it is
  // going for the right degree amount

  // While loop to do the spin
  if (referenceHeading < 0) {
    while (imuHeading <= referenceHeading) {
      MotorLF.spin(directionType::rev);
      MotorRF.spin(directionType::fwd);
      MotorLB.spin(directionType::rev);
      MotorRB.spin(directionType::fwd);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }

  } else {
    while (imuHeading <= referenceHeading) {
      MotorLF.spin(directionType::fwd);
      MotorRF.spin(directionType::rev);
      MotorLB.spin(directionType::fwd);
      MotorRB.spin(directionType::rev);
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
  MotorLB.setVelocity(power, velocityUnits::pct);
  MotorLF.setVelocity(power, velocityUnits::pct);
  MotorRB.setVelocity(power, velocityUnits::pct);
  MotorRF.setVelocity(power, velocityUnits::pct);

  MotorLB.spinFor(revs, rotationUnits::rev, false);
  MotorLF.spinFor(revs, rotationUnits::rev, false);
  MotorRB.spinFor(revs, rotationUnits::rev, false);
  MotorRF.spinFor(revs, rotationUnits::rev, true);
}
void moveIn(double inches, int poder) {
  double revs = inches / (4.000 * M_PI);
  MotorLB.setVelocity(poder, velocityUnits::pct);
  MotorLF.setVelocity(poder, velocityUnits::pct);
  MotorRB.setVelocity(poder, velocityUnits::pct);
  MotorRF.setVelocity(poder, velocityUnits::pct);

  MotorLB.spinFor(revs, rotationUnits::rev, false);
  MotorLF.spinFor(revs, rotationUnits::rev, false);
  MotorRB.spinFor(revs, rotationUnits::rev, false);
  MotorRF.spinFor(revs, rotationUnits::rev, true);
}
void balance(double power){
  while(imu.pitch(rotationUnits::deg) > 5 || imu.pitch(rotationUnits::deg) < -5){
    MotorLB.setVelocity(power, velocityUnits::pct);
    MotorLF.setVelocity(power, velocityUnits::pct);
    MotorRB.setVelocity(power, velocityUnits::pct);
    MotorRF.setVelocity(power, velocityUnits::pct);
    MotorLF.spin(directionType::fwd);
    MotorRF.spin(directionType::fwd);
    MotorLB.spin(directionType::fwd);
    MotorRB.spin(directionType::fwd);
  }
  driveStop(true);
}
void turnNoIMU(double referenceHeading, double power, double kp, double ki, double kd){
  double lastError = 0;
  float integralSum = 0;
  timer period = timer();
  MotorLF.resetRotation();
  if (referenceHeading < 0) {
    while ((double)MotorLB.rotation(rotationUnits::rev) <= (double)angleCalc(referenceHeading)) {
      period.reset();
      double error;
      double imuHeading = MotorLB.rotation(rotationUnits::rev);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      MotorLF.setVelocity(100 * output, velocityUnits::pct);
      MotorLB.setVelocity(100 * output, velocityUnits::pct);
      MotorRB.setVelocity(100 * output, velocityUnits::pct);
      MotorRF.setVelocity(100 * output, velocityUnits::pct);

      MotorLF.spin(directionType::rev);
      MotorRF.spin(directionType::fwd);
      MotorLB.spin(directionType::rev);
      MotorRB.spin(directionType::fwd);

      lastError = error;
      Controller1.Screen.clearScreen();
      Controller1.Screen.print(imu.angle(rotationUnits::deg));
      this_thread::sleep_for(10);
    }
  }
   else {
    while ((double)MotorLB.rotation(rotationUnits::rev) >= (double)angleCalc(referenceHeading)){
      period.reset();
      double error;
      double imuHeading = imu.angle(rotationUnits::deg);
      error = referenceHeading - imuHeading;
      double derivative = (error - lastError) / (double)period.value();
      integralSum = integralSum + (error * (double)period.value());
      double output = clip((kp * error) + (ki * integralSum) + (kd * derivative), -1, 1);

      MotorLF.setVelocity(100 * output, velocityUnits::pct);
      MotorLB.setVelocity(100 * output, velocityUnits::pct);
      MotorRB.setVelocity(100 * output, velocityUnits::pct);
      MotorRF.setVelocity(100 * output, velocityUnits::pct);

      MotorLF.spin(directionType::fwd);
      MotorRF.spin(directionType::rev);
      MotorLB.spin(directionType::fwd);
      MotorRB.spin(directionType::rev);

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
    MotorLF.rotateTo((double)angleCalc(angle), rotationUnits::rev, poder, velocityUnits::pct, false);
    MotorRF.rotateTo((double)angleCalc(angle) , rotationUnits::rev, -poder, velocityUnits::pct, false);
    MotorLB.rotateTo((double)angleCalc(angle), rotationUnits::rev, poder, velocityUnits::pct, false);
    MotorRB.rotateTo((double)angleCalc(angle) , rotationUnits::rev, -poder, velocityUnits::pct, true);
  }
  else if(angle < 0){
    MotorLF.rotateTo((double)angleCalc(angle), rotationUnits::rev, -poder, velocityUnits::pct, false);
    MotorRF.rotateTo((double)angleCalc(angle) , rotationUnits::rev, poder, velocityUnits::pct, false);
    MotorLB.rotateTo((double)angleCalc(angle), rotationUnits::rev, -poder, velocityUnits::pct, false);
    MotorRB.rotateTo((double)angleCalc(angle) , rotationUnits::rev, poder, velocityUnits::pct, true);
  }
  driveStop(true);
}

void moveInches(double inches, double poder){
  MotorLF.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, false);
  MotorRF.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, false);
  MotorLB.rotateTo((inches / (4 * M_PI)), rotationUnits::rev, poder, velocityUnits::pct, false);
  MotorRB.rotateTo((inches / (4 * M_PI)) , rotationUnits::rev, poder, velocityUnits::pct, true);
}
#endif