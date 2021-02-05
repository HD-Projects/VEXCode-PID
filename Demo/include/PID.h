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

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;

#ifndef PID_OBJECT
#define PID_OBJECT

/*----------------------------------------------------------------------------*/
/** @file     PID.h                                                           */
/** @brief    PID class header                                               */
/*----------------------------------------------------------------------------*/
/**
 * @var		namespace	PID
 * @global
 */
namespace PID {
/**
 * PID.
 *
 * @author	ad101-lab
 * @since	v0.0.1
 * @version	v1.0.0	Tuesday, August 25th, 2020.
 */
class PID {
public:
  // Init values

  /**
   * kP tuning vairable
   */
  double kP = 0;

  /**
   * kI tuning vairable
   */
  double kI = 0;

  /**
   * kD tuning vairable
   */
  double kD = 0;

  /**
   * Boolean to control if the loop is running or not
   */
  bool enabled = false;

  /**
   * A boolean to keep keep track for reseting the sensors
   */
  bool resetEncoder = false;

  /**
   * Initializes a blank motor on port 21 because
   * VEXCode doesn't have a defult constructor for
   * motors or motorgroups.
   */
  vex::motor_group PIDMotor = motor_group();

  // Advanced

  int sleepTime;         // The time that the loop sleeps each time.
  int maxIntegral = 300; // Max out integral to prevent oscillattion
  int integralBound = 3;

  int error;         // sensorValue - DesiredValue : Positional
  int prevError = 0; // Error the previous loop
  int derivative;    //
  int totalError = 0;

  // Initialize distance
  int desiredValue;

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 12:58:47
   * @drawer_cat{constructor}
   * @brief: A constructor for a drive PID
   * @param P Proportional
   * @param I Integral
   * @param D Derivative
   * @param Motor The motor_group to move when activeated
   */
  PID(double P, double I, double D, motor_group PID_Motor) {
    PIDMotor = PID_Motor;

    this->kP = P;
    this->kI = I;
    this->kD = D;
  }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:28:32
   * @brief Another constructor for PID with motor instead of motor_group
   * @param P kP varable
   * @param I kI varable
   * @param D kD varable
   */
  PID(double P, double I, double D, motor PID_Motor) {
    PIDMotor = motor_group(PID_Motor);

    this->kP = P;
    this->kI = I;
    this->kD = D;
  }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:31:16
   * @brief Enable the object of PID
   */
  void enable() { this->enabled = true; }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:31:16
   * @brief Disable the object of PID
   */
  void disable() { this->enabled = false; }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:31:16
   * @brief Reset the sensors
   */
  void reset() { this->resetEncoder = true; }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:33:44
   * @brief Move a selected distance
   * @param distance Distance to move
   */
  void move(double distance) {
    this->reset();
    this->desiredValue = distance;
  }

  int staticLoop(void *loopingPID) {
    // Check if pointer is null to remove unneeded errors
    if (loopingPID == NULL)
      return 0;

    PID *PIDobj = static_cast<PID *>(loopingPID);

    PIDobj->loop();

    return 1;
  }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:36:48
   * @brief: Call this function in a vex::task instance to run the PID
   */
  int loop();
};

/**
 * DrivePID.
 *
 * @author ad101-lab
 * @since	v0.0.1
 * @version	v1.0.0	Tuesday, August 25th, 2020.
 */
class DrivePID {
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
  inertial turning = inertial(PORT11);

  // Varable to keep track of "on" state
  bool enabled = false;

  // Reset Sensor Reset Value
  bool resetSensors = false;

  // Advanced

  int sleepTime;         // The time that the loop sleeps each time.
  int maxIntegral = 300; // Max out integral to prevent
  int maxTurnIntegral = 300;
  int integralBound = 3;

  int error;         // sensorValue - DesiredValue : Positional
  int prevError = 0; // Error the previous loop
  int derivative;    //
  int totalError = 0;

  int turnError;         // sensorValue - DesiredValue : Positional
  int turnPrevError = 0; // Error the previous loop
  int turnDerivative;    //
  int turnTotalError = 0;

  // Initialize distance
  int desiredValue;
  int desiredTurnValue;

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 12:58:47
   * @drawer_cat{constructor}
   * @brief: A constructor for a drive PID
   * @param P Proportional
   * @param I Integral
   * @param D Derivative
   * @param turnP Turn Proportional
   * @param turnI Turn Integral
   * @param turnD Turn Derivative
   * @param left Left motor_group for the left side of this drivetrain
   * @param right Right motor_group for the left side of this drivetrain
   * @param turnIntertial Default=21 inertial that you use for turning
   * @param wheelDiameter (Optional) the diameter of your wheels
   * @param sleep (Optional) for advanced users change the sleep time per
   loop
   */
  DrivePID(double P, double I, double D, double turnP, double turnI,
           double turnD, motor_group left, motor_group right, int turnIntertial,
           double wheelDiameter, int sleep = 20) {
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
  }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 12:58:47
   * @drawer_cat{constructor}
   * @brief: A constructor for a drive PID Sets bot turn and lateral kP, kI
   and kD to the same thing
   * @param P Proportional
   * @param I Integral
   * @param D Derivative
   * @param left Left motor_group for the left side of this drivetrain
   * @param right Right motor_group for the left side of this drivetrain
   * @param turnIntertial inertial that you use for turning
   * @param wheelDiameter (Optional) the diameter of your wheels
   * @param sleep (Optional) for advanced users change the sleep time per
   loop
   */
  DrivePID(double P, double I, double D, motor_group left, motor_group right,
           int turnIntertial, double wheelDiameter, int sleep = 20) {
    // Set P, I and D
    kP, turnkP = P;
    kI, turnkI = I;
    kD, turnkD = D;

    // Set motors
    leftMotors = left;
    rightMotors = right;
  }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 12:58:47
   * @drawer_cat{constructor}
   * @brief: A constructor for a drive Still need to set all of the values.
   */
  DrivePID() {}

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:31:16
   * @brief Enable the object of PID
   */
  void enable() { enabled = true; }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:31:16
   * @brief Disable the object of PID
   */
  void disable() { enabled = false; }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:31:16
   * @brief Reset the sensors
   */
  void reset() { resetSensors = true; }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:33:44
   * @brief Move a selected distance
   * @param distance Distance to move
   */
  void move(double distance) {
    // Reset the drive sensors
    resetSensors = true;

    // Set desired value
    desiredValue = distance;
  }

  void turn(double degrees) {
    // Reset the drive sensors
    resetSensors = true;

    // Set desired value
    desiredTurnValue = degrees;
  }

  /**
   * @Author: ad101-lab
   * @Date: 2020-08-25 14:36:48
   * @brief: Call this function in a vex::task instance to run the PID
   */
  int loop();
};
}; // namespace PID

#endif
