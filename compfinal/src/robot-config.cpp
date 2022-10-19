#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor MotorLB = motor(PORT16, ratio18_1, true);
motor MotorLF = motor(PORT18, ratio18_1, false);
motor MotorRB = motor(PORT17, ratio18_1, false);
motor MotorRF = motor(PORT19, ratio18_1, true);
motor ArmL = motor(PORT4, ratio36_1, false);
motor ArmR = motor(PORT2, ratio36_1, true);
motor ArmB = motor(PORT14, ratio36_1);
inertial imu = inertial(PORT13);
motor Claw = motor(PORT21, ratio18_1, true);
motor_group arm = motor_group(ArmL, ArmR);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  while (imu.isCalibrating()) { // wait for IMU to calibrate before runnin auton
    vexDelay(100);
  }
}