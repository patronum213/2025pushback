#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotor1 = motor(PORT1, ratio6_1, true);
motor LeftMotor2 = motor(PORT2, ratio6_1, true);
motor LeftMotor3 = motor(PORT3, ratio6_1, true);
motor RightMotor1 = motor(PORT11, ratio6_1, false);
motor RightMotor2 = motor(PORT12, ratio6_1, false);
motor RightMotor3 = motor(PORT13, ratio6_1, false);
motor IntakeMotor = motor(PORT9, ratio18_1, true);
motor OuttakeMotor = motor(PORT10, ratio18_1, true);



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}