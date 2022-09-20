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
void driveStop(bool holding){
  if(holding){
    MotorRB.stop(hold);
    MotorLB.stop(hold);
    MotorLF.stop(hold);
    MotorRF.stop(hold);
  }
  else{
    MotorRB.stop();
    MotorLB.stop();
    MotorLF.stop();
    MotorRF.stop();
  }
}

void setDriveVelocity(float velocity) {
  MotorLF.setVelocity(velocity, velocityUnits::pct);
  MotorLB.setVelocity(velocity, velocityUnits::pct);
  MotorRB.setVelocity(velocity, velocityUnits::pct);
  MotorRF.setVelocity(velocity, velocityUnits::pct);
}
void driveCorrection(float velocity, int dir){
  setDriveVelocity(velocity);
  MotorRF.spinTo(-0.1, rotationUnits::rev, false);
  MotorRB.spinTo(-0.1, rotationUnits::rev, false);
  MotorLF.spinTo(0.1, rotationUnits::rev, false);
  MotorLB.spinTo(0.1, rotationUnits::rev, true);
}

double clip(double number, double min, double max) {
  if (number < min) {
    number = min;
  } else if (number > max) {
    number = max;
  }
  return number;
}

void turnOnPID(double gyroRequestedValue, double MaxspeedinRPM) { //no params for PID consts
  float gyroSensorCurrentValue; //current sensor value for IMU
  float dir;
  float gyroError; //error
  float gyroDrive; //output var
  float lastgyroError; //las error
  float gyroP; //P var
  float gyroD; //D var
  //consts
  const float gyro_Kp = 0.573;
  const float gyro_Ki = 0.4;
  const float gyro_Kd = 0.18;

  int TimeExit = 0;
  double Threshold = 1.5; //threshold in degs for turning
  while (1) {
    gyroSensorCurrentValue = imu.rotation(vex::rotationUnits::deg); //assign IMU val to var
    gyroError = gyroRequestedValue - gyroSensorCurrentValue; //set gyro error

    if (gyroError < Threshold and gyroError > -Threshold) { //if the gyro reports requested value, break 
      break;
    } else if (TimeExit == 10000) {
      driveStop(true);
      break;
    } else {
      TimeExit = 0;
    }

    gyroP = (gyro_Kp * gyroError); //calculate P
    static float gyroI = 0; //set I
    gyroI += gyroError * gyro_Ki;
    if (gyroI > 1) { //clip if value is out of bounds
      gyroI = 1;
    }
    if (gyroI < -1) {
      gyroI = -1;
    }
    gyroD = (gyroError - lastgyroError) * gyro_Kd; //update D value 
    gyroDrive = gyroP + gyroI + gyroD; //set output var to P+I+D

    if (gyroDrive > MaxspeedinRPM) { //if requested value is (-) then turn CC, else, turn C (+)
      gyroDrive = MaxspeedinRPM;
    }
    if (gyroDrive < -MaxspeedinRPM) {
      gyroDrive = -MaxspeedinRPM;
    }
    int powerValue = gyroDrive;
    bool breakSwitch = true;

    if(breakSwitch){
      if(powerValue > 0){
        dir = 1;
      }
      else if(powerValue < 0){
        dir = -1;
      }
      breakSwitch = false;
    }

    MotorRF.spin(vex::directionType::rev, (powerValue), vex::velocityUnits::rpm); //spin motors to output var
    MotorLF.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm); //always equal to or lower than max speed specified
    MotorRB.spin(vex::directionType::rev, (powerValue), vex::velocityUnits::rpm);
    MotorLB.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    lastgyroError = gyroError; //set the last error for loop iteration
    wait(10, vex::timeUnits::msec); //wait for no wasted resources
  }
  driveStop(false); //stop the drive
  driveCorrection(20, dir);
  driveStop(true);
}

void driveOnPID(double distance, double MaxspeedinRPM, float kP, float kI, float kD) { 
  MotorRB.resetRotation();
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
    encoderValue = MotorRB.rotation(vex::rotationUnits::deg);
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
    MotorRF.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    MotorLF.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    MotorRB.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    MotorLB.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    lastError = error;
    wait(10, vex::timeUnits::msec);
  }
  driveStop(true);
}
#endif