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
    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;

    PIDMotor.spin(forward, lateralMotorPower, voltageUnits::volt);
  }
  return enabled ? 1 : 0;
}
} // namespace PID
