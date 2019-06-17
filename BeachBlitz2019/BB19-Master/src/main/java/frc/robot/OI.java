/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Input.LogitechAttack3Joystick;
import frc.robot.Input.LogitechF310;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static final LogitechAttack3Joystick LeftStick = new LogitechAttack3Joystick(RobotMap.LeftStickPort);
  private static final LogitechAttack3Joystick RightStick = new LogitechAttack3Joystick(RobotMap.RightStickPort);
  private static final LogitechF310 Gamepad = new LogitechF310(RobotMap.GamepadPort);

  public static double getLeftThrottleInput() {
    return LeftStick.getYAxis();
}
public static double getRightThrottleInput() {
    return RightStick.getYAxis();
}
public static double getLeftSteeringInput() {
    return LeftStick.getXAxis();
}
public static double getRightSteeringInput() {
    return RightStick.getXAxis();
}
public static double getLeftThrottleInputInverted() {
    return LeftStick.getYAxisInverted();
}
public static double getRightThrottleInputInverted() {
    return RightStick.getYAxisInverted();
}
public static double getLeftSteeringInputInverted() {
    return LeftStick.getXAxisInverted();
}
public static double getRightSteeringInputInverted() {
    return RightStick.getXAxisInverted();
}
}
