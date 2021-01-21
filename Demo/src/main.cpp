/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ad101-lab                                                 */
/*    Created:      Tue Aug 11 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// Copied from last years code
motor rightFWD =
motor(PORT12, ratio18_1, true); // Sets up the drivetrain motors(rightFWD)
motor leftFWD = motor(PORT13, ratio18_1, false);  // Left FWD
motor rightBack = motor(PORT14, ratio18_1, true); // Right Back
motor leftBack = motor(PORT17, ratio18_1, false); // Left Back

// Create a simple motor group to control all of them
motor_group driveMotors = motor_group(rightFWD, leftFWD, rightBack, leftBack);

controller pidTuner = controller(primary);

// PID object
PID::PID test_PID = PID::PID(1.0, 0.0, 0.0, driveMotors);

// Function to run non static methods
int loopFunc(void *loopPID) {
  if (loopPID == NULL)
    return 0;
  PID::PID *PIDobj = static_cast<PID::PID *>(loopPID);
  PIDobj->loop();
  return 1;
}

// Event callbacks for buttons
void upButton() {

}

void downButton() {

}

void resetButton() {

}

void runTest() {
  test_PID.disable();
  test_PID.reset();
}

void disable() {
  test_PID.disable();
}

int updatePidValues() {
  return 1;
}

int updateScreen() {
  while(1 == 1){
    pidTuner.Screen.clearScreen();

    // Sleep to save resources
    vex::task::sleep(100);
  }
  // return a defult value for the compliler although it is NEVER going
  // get to this becase 1 will always equal 1
  return 1;
}
// Function to setup the defined callbacks
void setupCallbacks(controller controller1) {
  controller1.ButtonUp.pressed(upButton);
  controller1.ButtonDown.pressed(downButton);
  controller1.ButtonA.pressed(runTest);
  controller1.ButtonX.pressed(resetButton);
  controller1.ButtonB.pressed(disable);
}

// Main loop
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // call the callback setup code
  setupCallbacks(pidTuner);

  // You have to use static functions with VEX tasks because VEXOS is written in C
  task PidLoop(loopFunc, &test_PID);
}
