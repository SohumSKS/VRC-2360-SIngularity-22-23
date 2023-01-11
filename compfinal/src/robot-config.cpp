#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor motorLB = motor(PORT16, ratio18_1, true);
motor motorLF = motor(PORT20, ratio18_1, true);
motor motorRB = motor(PORT17, ratio18_1, true);
motor motorRF = motor(PORT19, ratio18_1, true);

motor intakeF = motor(PORT1, ratio18_1, false);
motor intakeB = motor(PORT15, ratio18_1, false);

motor fly1 = motor(PORT11, ratio6_1, true);
motor fly2 = motor(PORT12, ratio6_1, false);
motor_group arm = motor_group(fly1, fly2);

digital_out indexer = digital_out( Brain.ThreeWirePort.B );
digital_out stringShooter1 = digital_out( Brain.ThreeWirePort.A );
digital_out stringShooter2 = digital_out( Brain.ThreeWirePort.C );

inertial imu = inertial(PORT13);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  while (imu.isCalibrating()) { // wait for IMU to calibrate before runnin auton
    vexDelay(100);
  }
}