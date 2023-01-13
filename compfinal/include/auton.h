#ifndef AUTON_H
#define AUTON_H
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       auton.h                                                   */
/*    Author:       Sohum Suthar                                              */
/*    Created:      Oct 10 2021                                               */
/*    Description:  Competition Program 2360S                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <vex_competition.h>

using namespace vex;
void driveStop(bool holding) {
  if (holding) {
    motorRB.stop(hold);
    motorLB.stop(hold);
    motorLF.stop(hold);
    motorRF.stop(hold);
  } else {
    motorRB.stop();
    motorLB.stop();
    motorLF.stop();
    motorRF.stop();
  }
}

void setDriveVelocity(float velocity) {
  motorLF.setVelocity(velocity, velocityUnits::pct);
  motorLB.setVelocity(velocity, velocityUnits::pct);
  motorRB.setVelocity(velocity, velocityUnits::pct);
  motorRF.setVelocity(velocity, velocityUnits::pct);
}
void driveCorrection(float velocity, int dir) {
  setDriveVelocity(velocity);
  motorRF.spinTo(-0.1, rotationUnits::rev, false);
  motorRB.spinTo(-0.1, rotationUnits::rev, false);
  motorLF.spinTo(0.1, rotationUnits::rev, false);
  motorLB.spinTo(0.1, rotationUnits::rev, true);
}

double clip(double number, double min, double max) {
  if (number < min) {
    number = min;
  } else if (number > max) {
    number = max;
  }
  return number;
}

void turnOnPID(double gyroRequestedValue,
               double MaxspeedinRPM) { // no params for PID consts
  float gyroSensorCurrentValue;        // current sensor value for IMU
  float dir;
  float gyroError;     // error
  float gyroDrive;     // output var
  float lastgyroError; // las error
  float gyroP;         // P var
  float gyroD;         // D var
  // consts
  const float gyro_Kp = 0.973;
  const float gyro_Ki = 0.1;
  const float gyro_Kd = 0.95;

  int TimeExit = 0;
  double Threshold = 1.5; // threshold in degs for turning
  while (1) {
    gyroSensorCurrentValue =
        imu.rotation(vex::rotationUnits::deg); // assign IMU val to var
    gyroError = gyroRequestedValue - gyroSensorCurrentValue; // set gyro error

    if (gyroError < Threshold and
        gyroError > -Threshold) { // if the gyro reports requested value, break
      break;
    } else if (TimeExit == 10000) {
      driveStop(true);
      break;
    } else {
      TimeExit = 0;
    }

    gyroP = (gyro_Kp * gyroError); // calculate P
    static float gyroI = 0;        // set I
    gyroI += gyroError * gyro_Ki;
    if (gyroI > 1) { // clip if value is out of bounds
      gyroI = 1;
    }
    if (gyroI < -1) {
      gyroI = -1;
    }
    gyroD = (gyroError - lastgyroError) * gyro_Kd; // update D value
    gyroDrive = gyroP + gyroI + gyroD;             // set output var to P+I+D

    if (gyroDrive > MaxspeedinRPM) { // if requested value is (-) then turn CC,
                                     // else, turn C (+)
      gyroDrive = MaxspeedinRPM;
    }
    if (gyroDrive < -MaxspeedinRPM) {
      gyroDrive = -MaxspeedinRPM;
    }
    int powerValue = gyroDrive;
    bool breakSwitch = true;

    if (breakSwitch) {
      if (powerValue > 0) {
        dir = 1;
      } else if (powerValue < 0) {
        dir = -1;
      }
      breakSwitch = false;
    }

    motorRF.spin(vex::directionType::rev, (powerValue),
                 vex::velocityUnits::rpm); // spin motors to output var
    motorLF.spin(vex::directionType::fwd, (-powerValue),
                 vex::velocityUnits::rpm); // always equal to or lower than max
                                           // speed specified
    motorRB.spin(vex::directionType::rev, (powerValue),
                 vex::velocityUnits::rpm);
    motorLB.spin(vex::directionType::fwd, (-powerValue),
                 vex::velocityUnits::rpm);
    lastgyroError = gyroError;      // set the last error for loop iteration
    wait(10, vex::timeUnits::msec); // wait for no wasted resources
  }
  // driveStop(false); //stop the drive
  // driveCorrection(20, dir);
  driveStop(true);
}

void driveOnPID(double distance, double MaxspeedinRPM, float kP, float kI,
                float kD) {
  motorRB.resetRotation();
  float degs = (distance / (4 * M_PI)) * 360;
  float encoderValue;
  float error;
  float Drive;
  float lastError;
  float P;
  float D;

  const float Kp = kP;
  const float Ki = kI;
  const float Kd = kD;

  int TimeExit = 0;
  double Threshold = 2.5;
  while (1) {
    encoderValue = motorRB.rotation(vex::rotationUnits::deg);
    Brain.Screen.setCursor(3, 1);
    error = degs - encoderValue;

    if (error < Threshold and error > -Threshold) {
      break;
    } else if (TimeExit == 10000) {
      Brain.Screen.clearScreen();
      driveStop(true);
      break;
    } else {
      TimeExit = 0;
    }

    P = (Kp * error);
    static float I = 0;
    I += error * Ki;
    if (I > 1) {
      I = 1;
    }
    if (I < -1) {
      I = -1;
    }
    D = (error - lastError) * Kd;
    Drive = P + I + D;

    if (Drive > MaxspeedinRPM) {
      Drive = MaxspeedinRPM;
    }
    if (Drive < -MaxspeedinRPM) {
      Drive = -MaxspeedinRPM;
    }
    int powerValue = Drive;
    motorRF.spin(vex::directionType::fwd, (powerValue),
                 vex::velocityUnits::rpm);
    motorLF.spin(vex::directionType::fwd, (-powerValue),
                 vex::velocityUnits::rpm);
    motorRB.spin(vex::directionType::fwd, (powerValue),
                 vex::velocityUnits::rpm);
    motorLB.spin(vex::directionType::fwd, (-powerValue),
                 vex::velocityUnits::rpm);
    lastError = error;
    wait(10, vex::timeUnits::msec);
  }
  driveStop(true);
}

void rightAngle() {
  int ninety = 90;
  double remainder = (int(imu.rotation(vex::rotationUnits::deg)) % ninety);
  turnOnPID(imu.rotation(vex::rotationUnits::deg) - remainder, 100);
}

void strafeOnPID(double distance, double MaxspeedinRPM, float kP, float kI,
                 float kD) {
  motorRB.resetRotation();
  float degs = (distance / (4 * M_PI)) * 360;
  float encoderValue;
  float error;
  float Drive;
  float lastError;
  float P;
  float D;

  const float Kp = kP;
  const float Ki = kI;
  const float Kd = kD;

  int TimeExit = 0;
  double Threshold = 2.5;
  while (1) {
    encoderValue = motorRB.rotation(vex::rotationUnits::deg);
    Brain.Screen.setCursor(3, 1);
    error = degs - encoderValue;

    if (error < Threshold and error > -Threshold) {
      break;
    } else if (TimeExit == 10000) {
      Brain.Screen.clearScreen();
      driveStop(true);
      break;
    } else {
      TimeExit = 0;
    }

    P = (Kp * error);
    static float I = 0;
    I += error * Ki;
    if (I > 1) {
      I = 1;
    }
    if (I < -1) {
      I = -1;
    }
    D = (error - lastError) * Kd;
    Drive = P + I + D;

    if (Drive > MaxspeedinRPM) {
      Drive = MaxspeedinRPM;
    }
    if (Drive < -MaxspeedinRPM) {
      Drive = -MaxspeedinRPM;
    }
    int powerValue = Drive;

    int sideways = powerValue;

    motorRF.spin(vex::forward, -sideways, vex::velocityUnits::rpm);
    motorLF.spin(vex::forward, -sideways, vex::velocityUnits::rpm);
    motorRB.spin(vex::forward, sideways, vex::velocityUnits::rpm);
    motorLB.spin(vex::forward, sideways, vex::velocityUnits::rpm);

    lastError = error;
    wait(10, vex::timeUnits::msec);
  }
  driveStop(true);
}

void shoot(int quantity, bool prefly) {
  if (!prefly) {
    fly1.spin(directionType::fwd, 100, percentUnits::pct);
    fly2.spin(directionType::fwd, 100, percentUnits::pct);
    wait(4000, timeUnits::msec);
  }

  while (quantity >= 1) {
    indexer.set(true);
    wait(250, timeUnits::msec);
    indexer.set(false);
    wait(400, timeUnits::msec);

    quantity--;
  }
  if (!prefly) {
    fly1.stop();
    fly2.stop();
  }
}

void fly(bool state) {
  fly1.setVelocity(42, percentUnits::pct);
  fly2.setVelocity(42, percentUnits::pct);
  if (state) {
    fly1.spin(directionType::fwd, 100, percentUnits::pct);
    fly2.spin(directionType::fwd, 100, percentUnits::pct);
  } else {
    fly1.stop();
    fly2.stop();
  }
}
#endif
