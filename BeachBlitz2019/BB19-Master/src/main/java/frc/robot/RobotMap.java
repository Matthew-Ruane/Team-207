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

  public static final int LeftStickPort = 0;
  public static final int RightStickPort = 1;
  public static final int GamepadPort = 2;

  public static final int mDrive_Left_A_ID = 3;
  public static final int mDrive_Left_B_ID = 5;
  public static final int mDrive_Right_A_ID = 4;
  public static final int mDrive_Right_B_ID = 6;
  
}
