/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) HD Open Source Projects 2020, All rights reserved.        */
/*                                                                            */
/*    Module:     PID.h                                                       */
/*    Author:     ad101-lab                                                   */
/*    Created:    25 August 2020                                              */
/*                                                                            */
/*    Revisions:                                                              */
/*                v0.0.1     TBD - Initial release                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <cmath>

using namespace vex;

/** 
 * @Author: ad101-lab 
 * @Date: 2020-08-25 13:41:44 
 * @brief:  
 * @param:  
 */

namespace PID {
  /**
   * @var		object	distance
   */
  enum distance{
    cm = 1,
    mm = 1,
    in = 1,
  };
  /**
   * PID.
   *
   * @author	ad101-lab
   * @since	v0.0.1
   * @version	v1.0.0	Wednesday, August 26th, 2020.
   */
  class PID {
    //Pasted from a C++ resource
    double signnum_c(double x) {
      if (x > 0.0) return 1.0;
      if (x < 0.0) return -1.0;
      return x;
    }
    public:
      /**
       * Init values
       *
       * @var		integer	kP
       */
      double kP = 0;

      /**
       * @var		integer	kI
       */
      double kI = 0;

      /**
       * @var		integer	kD
       */
      double kD = 0;

      /**
       * @var		bool	enabled
       */
      bool enabled = false;

      /**
       * @var		bool	resetEncoder
       */
      bool resetEncoder = false;

      /**
       * motor Init
       *
       * @var		motor	PIDMotor
       */
      motor PIDMotor = motor(21);

      // Advanced

      /**
       * The time that the loop sleeps each time.
       *
       * @var		int	sleepTime
       */
      int sleepTime;
      
      /**
       * Max out integral to prevent oscillattion
       *
       * @var		integer	maxIntegral
       */
      int maxIntegral = 300;

      /**
       * @var		integer	integralBound
       */
      int integralBound = 3;

      /**
       * sensorValue - DesiredValue : Positional
       *
       * @var		int	error
       */
      int error; 

      /**
       * Error the previous loop
       *
       * @var		integer	prevError
       */
      int prevError = 0; 

      /**
       * @var		int	derivative
       */
      int derivative; 

      /**
       * @var		integer	totalError
       */
      int totalError = 0;

      /**
       * Initialize distance
       *
       * @var		int	desiredValue
       */
      int desiredValue;

      /**
       * Initialize
       *
       * @var		mixed	PID(doubl
       */
      PID(double P, double I, double D, motor_group PID_Motor){
        motor_group PIDMotor = PID_Motor;

        kP = P;
        kI = I;
        kD = D;
      }

      /**
       * PID function that will set all Varables
       *
       * @var		mixed	PID(doubl
       */
      PID(double P, double I, double D, motor PID_Motor){
        motor PIDMotor = PID_Motor;
        
        // Set Varables
        kP = P;
        kI = I;
        kD = D;
      }

      /**
       * PID loop without Varables
       * Still have to set motor and
       * kP kI and kD values
       *
       * @var		object	PID()
       */
      PID(){
        // Set Varables
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
      }

      /**
       * Enable the PID
       * Still have to call loop
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @return	void
       */
      void enable(){
        enabled = true;
      }

      /**
       * Disables the PID
       * also ends the loop
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @return	void
       */
      void disable(){
        enabled = false;
      }

      /**
       * Resets the Encoders
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @return	void
       */
      void reset(){
        resetEncoder = true;
      }

      /**
       * Move forward degrees
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @param	double	distance	
       * @return	void
       */
      void move(double distance) {
        reset();
        desiredValue = distance;
      }

      int loop(){

        /**
         * Run only when the enabled varable is true.
         *
         * @var		object	while(enabled)
         */
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
  /**
   * DrivePID.
   *
   * @author	ad101-lab
   * @since	v0.0.1
   * @version	v1.0.0	Wednesday, August 26th, 2020.
   */
  class DrivePID {
    //Pasted from a C++ resource
    double signnum_c(double x) {
      if (x > 0.0) return 1.0;
      if (x < 0.0) return -1.0;
      return x;
    }

    public: 

      /**
       * Create doubles for P, I and D
       *
       * @var		double	kP
       */
      double kP;

      /**
       * @var		double	kI
       */
      double kI;

      /**
       * @var		double	kD
       */
      double kD;

      /**
       * turning
       *
       * @var		double	turnkP
       */
      double turnkP;

      /**
       * @var		double	turnkI
       */
      double turnkI;
      
      /**
       * @var		double	turnkD
       */
      double turnkD;

      /**
       * Left and Right motorgroups
       *
       * @var		motor_group	leftMotors
       */
      motor_group leftMotors;
      
      /**
       * @var		motor_group	rightMotors
       */
      motor_group rightMotors;

      /**
       * Sensors
       *
       * @var		inertial	turning
       */
      inertial turning = inertial(21);

      /**
       * Varable to keep track of "on" state
       *
       * @var		bool	enabled
       */
      bool enabled = false;

      /**
       * Reset Sensor Reset Value
       *
       * @var		bool	resetSensors
       */
      bool resetSensors = false;

      // Advanced

      /**
       * The time that the loop sleeps each time.
       *
       * @var		int	sleepTime
       */
      int sleepTime; 

      /**
       * Max out integral to prevent
       *
       * @var		integer	maxIntegral
       */
      int maxIntegral = 300;  

      /**
       * @var		integer	maxTurnIntegral
       */
      int maxTurnIntegral = 300;

      /**
       * @var		integer	integralBound
       */
      int integralBound = 3;

      /**
       * sensorValue - DesiredValue : Positional
       *
       * @var		int	error
       */
      int error;

      /**
       * Error the previous loop
       *
       * @var		integer	prevError
       */
      int prevError = 0;

      /**
       * @var		int	derivative
       */
      int derivative; 

      /**
       * @var		integer	totalError
       */
      int totalError = 0;

      // sensorValue - DesiredValue : Positional
      int turnError;

      // Error the previous loop
      int turnPrevError = 0;

      /**
       * @var		int	turnDerivative
       */
      int turnDerivative;

      /**
       * @var		integer	turnTotalError
       */
      int turnTotalError = 0;

      /**
       * Initialize distance
       *
       * @var		int	desiredValue
       */
      int desiredValue;

      /**
       * @var		int	desiredTurnValue
       */
      int desiredTurnValue;

      /**
       * DrivePID contructor
       *
       * @var		mixed	DrivePID(doubl
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

      /**
       * Enable the PID
       * Still have to call loop
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @return	void
       */
      void enable(){
        enabled = true;
      }

      /**
       * Disables the PID
       * also ends the loop
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @return	void
       */
      void disable(){
        enabled = false;
      }

      /**
       * Resets the Encoders
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @return	void
       */
      void reset(){
        resetSensors = true;
      }

      /**
       * move.
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @param	float	distance	
       * @return double distance
       */
      double move(double distance) {
        // Reset the drive sensors
        resetSensors = true;

        // Set desired value
        desiredValue = distance;

        //Return Value
        return distance;
      }

      /**
       * Turn degrees
       *
       * @author	ad101-lab
       * @since	v0.0.1
       * @version	v1.0.0	Wednesday, August 26th, 2020.
       * @param	degrees	
       * @return	mixed
       */
      double turn(double degrees) {
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