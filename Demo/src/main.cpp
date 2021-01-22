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

task updateScreenTask;
task updatePidValuesTask;
task PidLoop;
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

int tuningChoice = 0;

const int kPchoice = 0;
const int kIchoice = 1;
const int kDchoice = 2;

bool testing = false;

// Event callbacks for buttons
void upButton() { tuningChoice++; }

void downButton() { tuningChoice--; }

void resetButton() {
  test_PID.kP = 0;
  test_PID.kI = 0;
  test_PID.kD = 0;
}

void runTest() {
  testing = true;
  test_PID.disable();
  test_PID.reset();
  test_PID.enable();
  task::sleep(1000);
  test_PID.desiredValue = 360;
  task::sleep(2000);
  test_PID.desiredValue = 0;
  task::sleep(2000);
  test_PID.reset();
  test_PID.disable();
  testing = false;
}

void disable() { test_PID.disable(); }

int updatePidValues() {
  double ammountChange =
      pidTuner.Axis3.value() > 10 ? pidTuner.Axis3.value() / 64 : 0;

  switch (tuningChoice % 3) {
  case kPchoice:
    test_PID.kP += ammountChange;
    break;
  case kIchoice:
    test_PID.kI += ammountChange;
    break;
  case kDchoice:
    test_PID.kD += ammountChange;
    break;
  }

  return 1;
}

int updateScreen() {
  while (1 == 1) {
    if (testing) {
      pidTuner.Screen.clearScreen();
      pidTuner.Screen.setCursor(1, 1);
      pidTuner.Screen.print("PID Testing");

      pidTuner.Screen.setCursor(2, 1);
      pidTuner.Screen.print("Desired Value: ");
      pidTuner.Screen.print(test_PID.desiredValue);

      pidTuner.Screen.setCursor(3, 1);
      pidTuner.Screen.print("Error: ");
      pidTuner.Screen.print(test_PID.error);

      pidTuner.Screen.setCursor(4, 1);
      pidTuner.Screen.print("Total Error: ");
      pidTuner.Screen.print(test_PID.totalError);


    } else {
      
      pidTuner.Screen.clearScreen();
      pidTuner.Screen.setCursor(1, 1);
      pidTuner.Screen.print("PID Tuning");

      // Print data for kP
      pidTuner.Screen.setCursor(2, 1);
      if (tuningChoice % 3 == kPchoice) {
        pidTuner.Screen.print("-> ");
      }
      pidTuner.Screen.print("kP: ");
      pidTuner.Screen.print(test_PID.kP);

      // Print data for kI
      pidTuner.Screen.setCursor(2, 1);
      if (tuningChoice % 3 == kIchoice) {
        pidTuner.Screen.print("-> ");
      }
      pidTuner.Screen.print("kI: ");
      pidTuner.Screen.print(test_PID.kI);

      // Print data for kD
      pidTuner.Screen.setCursor(2, 1);
      if (tuningChoice % 3 == kDchoice) {
        pidTuner.Screen.print("-> ");
      }
      pidTuner.Screen.print("kD: ");
      pidTuner.Screen.print(test_PID.kD);
    }
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

  // You have to use static functions with VEX tasks because VEXOS is written in
  // C
  task updateScreenTask(updateScreen);
  task updatePidValuesTask(updatePidValues);
  task PidLoop(loopFunc, &test_PID);
}
