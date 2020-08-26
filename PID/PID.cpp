#include "vex.h"
#include <cmath>

using namespace vex;

/** 
 * @Author: ad101-lab 
 * @Date: 2020-08-25 13:41:44 
 * @brief:  
 * @param:  
 */

namespace PID {
  enum distance{
    cm = 1,
    mm = 1,
    in = 1,
  };
  class PID {
    //Pasted from a C++ resource
    double signnum_c(double x) {
      if (x > 0.0) return 1.0;
      if (x < 0.0) return -1.0;
      return x;
    }
    public:
      // Init values
      double kP = 0;
      double kI = 0;
      double kD = 0;

      bool enabled = false;

      bool resetEncoder = false;

      // motor Init
      motor PIDMotor = motor(21);

      // Advanced

      int sleepTime; // The time that the loop sleeps each time.
      int maxIntegral = 300; // Max out integral to prevent oscillattion
      int integralBound = 3;

      int error; // sensorValue - DesiredValue : Positional
      int prevError = 0; // Error the previous loop
      int derivative; // 
      int totalError = 0;

      // Initialize distance
      int desiredValue;

      // Initialize
      PID(double P, double I, double D, motor_group PID_Motor){
        motor_group PIDMotor = PID_Motor;

        kP = P;
        kI = I;
        kD = D;
      }

      PID(double P, double I, double D, motor PID_Motor){
        motor PIDMotor = PID_Motor;
        
        // Set Varables
        kP = P;
        kI = I;
        kD = D;
      }

      PID(){

        // Set Varables
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
      }

      // Enable the PID
      // Still have to call loop
      void enable(){
        enabled = true;
      }

      // Disables the PID
      // also ends the loop
      void disable(){
        enabled = false;
      }

      // Resets the Encoders
      void reset(){
        resetEncoder = true;
      }

      // Move forward degrees
      void move(double distance) {
        reset();
        desiredValue = distance;
      }

      int loop(){

        // Run only when the enabled varable is true.
        while(enabled){

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
          if(abs(error) < integralBound){
            totalError += error;
          } else {
            totalError = 0;
          }

          // Cap out integral to pprevent oscillate
          totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

          // Caculate Motorpower
          double lateralMotorPower = error * kP + derivative * kD + totalError * kI;

          PIDMotor.spin(forward, lateralMotorPower, voltageUnits::volt);
        }
        return enabled ? 1 : 0;
      }
  };
  class DrivePID {
    //Pasted from a C++ resource
    double signnum_c(double x) {
      if (x > 0.0) return 1.0;
      if (x < 0.0) return -1.0;
      return x;
    }

    public: 

      // Create doubles for P, I and D
      double kP;
      double kI;
      double kD;

      // turning
      double turnkP;
      double turnkI;
      double turnkD;

      // Left and Right motorgroups
      motor_group leftMotors;
      motor_group rightMotors;

      // Sensors
      inertial turning = inertial(21);

      // Varable to keep track of "on" state
      bool enabled = false;

      // Reset Sensor Reset Value
      bool resetSensors = false;

      // Advanced

      int sleepTime; // The time that the loop sleeps each time.
      int maxIntegral = 300; // Max out integral to prevent 
      int maxTurnIntegral = 300;
      int integralBound = 3;

      int error; // sensorValue - DesiredValue : Positional
      int prevError = 0; // Error the previous loop
      int derivative; // 
      int totalError = 0;

      int turnError; // sensorValue - DesiredValue : Positional
      int turnPrevError = 0; // Error the previous loop
      int turnDerivative; // 
      int turnTotalError = 0;

      // Initialize distance
      int desiredValue;
      int desiredTurnValue;

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 13:04:26 
       * @Desc: Drive PID Constructors
       */

      DrivePID(double P, double I, double D,
      double turnP, double turnI, double turnD,
      motor_group left, motor_group right, inertial turnIntertial,
      double wheelDiameter = 1, int sleep = 20) {
        // Set P, I and D
        kP = P;
        kI = I;
        kD = D;

        turnkP = turnP;
        turnkI = turnI;
        turnkD = turnD;

        // Set motors
        leftMotors = left;
        rightMotors = right;

        // Inertial setup
        turning = turnIntertial;
      }

      DrivePID(double P, double I, double D,
      motor_group left, motor_group right, inertial turnIntertial,
      double wheelDiameter = 1, int sleep = 20){
        // Set P, I and D
        kP, turnkP = P;
        kI, turnkI = I;
        kD, turnkD = D;

        // Set motors
        leftMotors = left;
        rightMotors = right;

        // Inertial setup
        turning = turnIntertial;
      }

      // Enable the PID
      // Still have to call loop
      void enable(){
        enabled = true;
      }

      // Disables the PID
      // also ends the loop
      void disable(){
        enabled = false;
      }

      // Resets the Encoders
      void reset(){
        resetSensors = true;
      }

      double move(float distance) {
        // Reset the drive sensors
        resetSensors = true;

        // Set desired value
        desiredValue = distance;

        //Return Value
        return distance;
      }

      double turn(float degrees) {
        // Reset the drive sensors
        resetSensors = true;

        // Set desired value
        desiredTurnValue = degrees;

        //Return Value
        return degrees;
      }

      int loop(){

        // Run only when the enabled varable is true.
        while(enabled){

          // Reset Drive Motors Function
          if (resetSensors) {
            resetSensors = false;
            leftMotors.setPosition(0,degrees);
            leftMotors.setPosition(0,degrees);
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
          if(abs(error) < integralBound){
            totalError += error;
          } else {
            totalError = 0;
          }

          // Cap out integral to pprevent oscillate
          totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

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
          if(abs(error) < integralBound){
            turnTotalError += turnError;
          } else {
            turnTotalError = 0;
          }

          // Set totalError
          turnTotalError = abs(turnTotalError) > maxIntegral ? signnum_c(turnTotalError) * maxIntegral : turnTotalError;

          // Set Turning ammount
          double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;

          // Set Previous Error to Error
          prevError = error;
          turnPrevError = turnError;

          // Move the motors

          leftMotors.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
          rightMotors.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);

          // Delay to prevent CPU usage
          task::sleep(sleepTime);
        }
        return 1;
      }
  };
};