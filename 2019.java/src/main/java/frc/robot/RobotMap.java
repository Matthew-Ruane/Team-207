/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
    public static final int mDrive_Left_A_ID = 3;
    public static final int mDrive_Left_B_ID = 5;
    public static final int mDrive_Left_C_ID = 7;
    public static final int mDrive_Right_A_ID = 4;
    public static final int mDrive_Right_B_ID = 6;
    public static final int mDrive_Right_C_ID = 8;

    public static final int mShooter_ID = 10;

    public static final int mElevator_Master_ID = 9;
    public static final int mElevator_Slave_ID = 11;

    public static final int mTray_Extend_ID = 1;
    public static final int mTray_Retract_ID = 1;
    public static final int mShift_High_ID = 2;
    public static final int mShift_Low_ID = 2;
    public static final int mTalons_Hold_ID = 3;
    public static final int mTalons_Release_ID = 3;

    public static final int mCargo_Loaded_Sensor_ID = 9;
    public static final int mHatch_Loaded_Sensor_ID = 0;

    public static final int mPCM_A = 2;
    public static final int mPCM_B = 1;

    public static final int LeftStickPort = 0;
    public static final int RightStickPort = 1;
    public static final int GamepadPort = 2;

    public static final int mUltrasonic_Ping_ID = 3;
    public static final int mUltrasonic_Echo_ID = 4;

    public static final boolean On = true;
    public static final boolean Off = false;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
