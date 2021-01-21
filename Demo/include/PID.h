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
      vex::motor PIDMotor = motor(21);

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
        vex::motor_group PIDMotor = PID_Motor;

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
        vex::motor PIDMotor = PID_Motor;

        this->kP = P;
        this->kI = I;
        this->kD = D;
      }

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Enable the object of PID
       */
      void enable() {
        this->enabled = true;
      }

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Disable the object of PID
       */
      void disable() {
        this->enabled = false;
      }

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Reset the sensors
       */
      void reset() {
        this->resetEncoder = true;
      }

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

          PID::PID *PIDobj = static_cast<PID::PID *>(loopingPID);

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
  //
  // Removing DrivePID temperaroly
  ///**
  // * DrivePID.
  // *
  // * @author ad101-lab
  // * @since	v0.0.1
  // * @version	v1.0.0	Tuesday, August 25th, 2020.
  // */
  //class DrivePID {
  //  public:
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 12:58:47 
  //     * @drawer_cat{constructor}
  //     * @brief: A constructor for a drive PID
  //     * @param P Proportional
  //     * @param I Integral
  //     * @param D Derivative
  //     * @param turnP Turn Proportional
  //     * @param turnI Turn Integral
  //     * @param turnD Turn Derivative
  //     * @param left Left motor_group for the left side of this drivetrain
  //     * @param right Right motor_group for the left side of this drivetrain
  //     * @param turnIntertial Default=21 inertial that you use for turning
  //     * @param wheelDiameter (Optional) the diameter of your wheels
  //     * @param sleep (Optional) for advanced users change the sleep time per loop
  //     */
  //    DrivePID( double P, double I, double D, double turnP, double turnI, double turnD, motor_group left, motor_group right, inertial turnIntertial, double wheelDiameter, int sleep = 20 );
  //    
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 12:58:47 
  //     * @drawer_cat{constructor}
  //     * @brief: A constructor for a drive PID Sets bot turn and lateral kP, kI and kD to the same thing
  //     * @param P Proportional
  //     * @param I Integral
  //     * @param D Derivative
  //     * @param left Left motor_group for the left side of this drivetrain
  //     * @param right Right motor_group for the left side of this drivetrain
  //     * @param turnIntertial inertial that you use for turning
  //     * @param wheelDiameter (Optional) the diameter of your wheels
  //     * @param sleep (Optional) for advanced users change the sleep time per loop
  //     */
  //    DrivePID( double P, double I, double D, motor_group left, motor_group right, inertial turnIntertial, double wheelDiameter, int sleep = 20 );
//
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 12:58:47 
  //     * @drawer_cat{constructor}
  //     * @brief: A constructor for a drive Still need to set all of the values.
  //     */
  //    DrivePID();
//
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 14:31:16 
  //     * @brief Enable the object of PID
  //     */
  //    void enable();
//
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 14:31:16 
  //     * @brief Disable the object of PID
  //     */
  //    void disable();
//
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 14:31:16 
  //     * @brief Reset the sensors
  //     */
  //    void reset();
//
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 14:33:44 
  //     * @brief Move a selected distance
  //     * @param distance Distance to move 
  //     */
  //    void move(double distance);
  //    
  //    /** 
  //     * @Author: ad101-lab 
  //     * @Date: 2020-08-25 14:36:48 
  //     * @brief: Call this function in a vex::task instance to run the PID 
  //     */
  //    int loop();
//
  //    /**
  //     * @var		double	kP
  //     */
  //    double kP;
//
  //    /**
  //     * @var		double	kI
  //     */
  //    double kI;
//
  //    /**
  //     * @var		double	kD
  //     */
  //    double kD;
//
  //    /**
  //     * @var		double	turnkP
  //     */
  //    double turnkP;
//
  //    /**
  //     * @var		double	turnkI
  //     */
  //    double turnkI;
//
  //    /**
  //     * @var		double	turnkD
  //     */
  //    double turnkD;
//
  //    /**
  //     * @var		motor_group	leftMotors
  //     */
  //    motor_group leftMotors;
//
  //    /**
  //     * @var		motor_group	rightMotors
  //     */
  //    motor_group rightMotors;
//
  //    /**
  //     * @var		inertial	turning
  //     */
  //    inertial turning;
//
  //    /**
  //     * @var		bool	enabled
  //     */
  //    bool enabled;
//
  //    /**
  //     * @var		bool	resetSensors
  //     */
  //    bool resetSensors;
//
//
  //    /**
  //     * @var		int	sleepTime
  //     */
  //    int sleepTime;
//
  //    /**
  //     * @var		int	maxIntegral
  //     */
  //    int maxIntegral;
//
  //    /**
  //     * @var		int	maxTurnIntegral
  //     */
  //    int maxTurnIntegral;
//
  //    /**
  //     * @var		int	integralBound
  //     */
  //    int integralBound;
//
  //    /**
  //     * sensorValue - DesiredValue : Positional
  //     *
  //     * @var		int	error
  //     */
  //    int error;
//
  //    /**
  //     * Error the previous loop
  //     *
  //     * @var		int	prevError
  //     */
  //    int prevError;
//
  //    /**
  //     * @var		int	derivative
  //     */
  //    int derivative;
//
  //    /**
  //     * @var		int	totalError
  //     */
  //    int totalError;
//
  //    /**
  //     * sensorValue - DesiredValue : Positional
  //     *
  //     * @var		int	turnError
  //     */
  //    int turnError;
//
  //    /**
  //     * Error the previous loop
  //     *
  //     * @var		integer	turnPrevError
  //     */
  //    int turnPrevError = 0;
//
  //    /**
  //     * @var		int	turnDerivative
  //     */
  //    int turnDerivative;
  //    /**
  //     * @var		integer	turnTotalError
  //     */
  //    int turnTotalError = 0;
//
  //    /**
  //     * Initialize distance
  //     *
  //     * @var		int	desiredValue
  //     */
  //    int desiredValue;
//
  //    /**
  //     * @var		int	desiredTurnValue
  //     */
  //    int desiredTurnValue;
  //};
};

#endif
