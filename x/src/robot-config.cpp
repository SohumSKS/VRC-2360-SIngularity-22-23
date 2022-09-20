#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller1;
motor left1 = motor(PORT14, ratio18_1, false);
motor left2 = motor(PORT3,  ratio18_1, false);
motor left3 = motor(PORT13,  ratio18_1, false);
motor right1 = motor(PORT10,  ratio18_1, false);
motor right2 = motor(PORT9,  ratio18_1, false);
motor right3 = motor(PORT1,  ratio18_1, false);
motor lift1 = motor(PORT16,  ratio18_1, false);
motor lift2 = motor(PORT6,  ratio18_1, false);
digital_out claw = digital_out(Brain.ThreeWirePort.B);
digital_out mob = digital_out(Brain.ThreeWirePort.A);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}