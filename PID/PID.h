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
      PID(double P, double I, double D, motor_group PID_Motor);

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:28:32 
       * @brief Another constructor for PID with motor instead of motor_group 
       * @param P kP varable
       * @param I kI varable
       * @param D kD varable
       */
      PID(double P, double I, double D, motor PID_Motor);

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Enable the object of PID
       */
      void enable();

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Disable the object of PID
       */
      void disable();

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Reset the sensors
       */
      void reset();

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:33:44 
       * @brief Move a selected distance
       * @param distance Distance to move 
       */
      void move(double distance);

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
   * @author	ad101-lab
   * @since	v0.0.1
   * @version	v1.0.0	Tuesday, August 25th, 2020.
   */
  class DrivePID {
    public:
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
       * @param sleep (Optional) for advanced users change the sleep time per loop
       */
      DrivePID( double P, double I, double D, double turnP, double turnI, double turnD, motor_group left, motor_group right, inertial turnIntertial, double wheelDiameter, int sleep = 20 );
      
      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 12:58:47 
       * @drawer_cat{constructor}
       * @brief: A constructor for a drive PID Sets bot turn and lateral kP, kI and kD to the same thing
       * @param P Proportional
       * @param I Integral
       * @param D Derivative
       * @param left Left motor_group for the left side of this drivetrain
       * @param right Right motor_group for the left side of this drivetrain
       * @param turnIntertial inertial that you use for turning
       * @param wheelDiameter (Optional) the diameter of your wheels
       * @param sleep (Optional) for advanced users change the sleep time per loop
       */
      DrivePID( double P, double I, double D, motor_group left, motor_group right, inertial turnIntertial, double wheelDiameter, int sleep = 20 );

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 12:58:47 
       * @drawer_cat{constructor}
       * @brief: A constructor for a drive Still need to set all of the values.
       */
      DrivePID();

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Enable the object of PID
       */
      void enable();

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Disable the object of PID
       */
      void disable();

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:31:16 
       * @brief Reset the sensors
       */
      void reset();

      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:33:44 
       * @brief Move a selected distance
       * @param distance Distance to move 
       */
      void move(double distance);
      
      /** 
       * @Author: ad101-lab 
       * @Date: 2020-08-25 14:36:48 
       * @brief: Call this function in a vex::task instance to run the PID 
       */
      int loop();
  };
};

#endif