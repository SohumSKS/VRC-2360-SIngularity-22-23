#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor MotorLB = motor(PORT11, ratio18_1, true);
motor MotorLF = motor(PORT12, ratio18_1, false);
motor MotorRB = motor(PORT19, ratio18_1, false);
motor MotorRF = motor(PORT20, ratio18_1, true);
motor ArmL = motor(PORT4, ratio36_1, false);
motor ArmR = motor(PORT2, ratio36_1, true);
motor ArmB = motor(PORT14, ratio36_1);
inertial imu = inertial(PORT13);
motor Claw = motor(PORT21, ratio18_1, true);
motor_group arm = motor_group(ArmL, ArmR);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

void vexcodeInit( void ) {

}