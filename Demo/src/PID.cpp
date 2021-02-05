#include "PID.h"
#include "vex.h"

namespace PID {

// Pasted from a C++ resource
double signnum_c(double x) {
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return x;
}

int PID::loop() {
  while (true) {
    /**
     * Run only when the enabled varable is true.
     *
     * @var		object	while(enabled)
     */
    while (enabled) {

      // Reset Drive Motors Function
      if (resetEncoder) {
        resetEncoder = false;
      }

      // Get motors positions
      int motorPosition = PIDMotor.position(degrees);

      /*

      Lateral Movement PID

      This is the PID Program that moves your robot back and forth

      */

      // Find error(Potential)
      error = motorPosition - desiredValue;

      // Derivative(Velocity)
      derivative = error - prevError;

      // Integral
      if (abs(error) < integralBound) {
        totalError += error;
      } else {
        totalError = 0;
      }

      // Cap out integral to prevent oscillate
      totalError = abs(totalError) > maxIntegral
                       ? signnum_c(totalError) * maxIntegral
                       : totalError;

      // Caculate Motorpower
      double lateralMotorPower = ((double)error * kP + (double)derivative * kD +
                                  (double)totalError * kI) /
                                 12;

      PIDMotor.spin(forward, -lateralMotorPower, voltageUnits::volt);

      task::sleep(50);
    }
    PIDMotor.spin(forward, 0, voltageUnits::volt);
    task::sleep(100);
  }
  return enabled ? 1 : 0;
}

int DrivePID::loop() {
  while (true == true) {
    
    // Run only when the enabled varable is true.
    while (enabled) {

      // Reset Drive Motors Function
      if (resetSensors) {
        resetSensors = false;
        leftMotors.setPosition(0, degrees);
        leftMotors.setPosition(0, degrees);
        turning.resetRotation();
      }

      // Get motors positions
      int leftMotorPosition = leftMotors.position(degrees);
      int rightMotorPosition = leftMotors.position(degrees);

      /*
      Lateral Movement PID
      This is the PID Program that moves your robot back and forth
      */

      // Mean out the motors
      int averageMotorPosition = (leftMotorPosition + rightMotorPosition) / 2;

      // Find error(Potential)
      error = averageMotorPosition - desiredValue;

      // Derivative(Velocity)
      derivative = error - prevError;

      // Integral
      if (abs(error) < integralBound) {
        totalError += error;
      } else {
        totalError = 0;
      }

      // Cap out integral to pprevent oscillate
      totalError = abs(totalError) > maxIntegral
                       ? signnum_c(totalError) * maxIntegral
                       : totalError;

      // Caculate Motorpower
      double lateralMotorPower = error * kP + derivative * kD + totalError * kI;

      /*
      Turn Movememnt PID
      This is the turning PID of the drivetrain
      */

      // Get sensor value
      int rotValue = turning.rotation();

      // Get turn error
      turnError = rotValue - desiredTurnValue;

      // Derivative(Velocity)
      turnDerivative = turnError - prevError;

      // Reduce oscillation
      if (abs(error) < integralBound) {
        turnTotalError += turnError;
      } else {
        turnTotalError = 0;
      }

      // Set totalError
      turnTotalError = abs(turnTotalError) > maxIntegral
                           ? signnum_c(turnTotalError) * maxIntegral
                           : turnTotalError;

      // Set Turning ammount
      double turnMotorPower = turnError * turnkP + turnDerivative * turnkD +
                              turnTotalError * turnkI;

      // Set Previous Error to Error
      prevError = error;
      turnPrevError = turnError;

      // Move the motors

      leftMotors.spin(forward, lateralMotorPower - turnMotorPower,
                      voltageUnits::volt);
      rightMotors.spin(forward, lateralMotorPower + turnMotorPower,
                       voltageUnits::volt);

      // Delay to prevent CPU usage
      task::sleep(sleepTime);
    }
  }
  return 1;
}
} // namespace PID
