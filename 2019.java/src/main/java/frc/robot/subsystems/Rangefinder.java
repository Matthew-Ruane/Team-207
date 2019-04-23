/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import frc.robot.RobotMap;

/**
 * Author Matt Ruane (207)
 */
public class Rangefinder extends Subsystem {

  private static final Rangefinder instance = new Rangefinder();

  public static Rangefinder getInstance() {
    return instance;
  }

  public static Ultrasonic mRangefinder = new Ultrasonic(RobotMap.mUltrasonic_Ping_ID, RobotMap.mUltrasonic_Echo_ID, Unit.kInches);
  public static double kDistance = mRangefinder.getRangeInches();
  public static SendableBuilder RangefinderDisplay;

  public Rangefinder() {
    mRangefinder.setEnabled(true);
    mRangefinder.initSendable(RangefinderDisplay);
  }

  public static double getDistance() {
    return kDistance;
  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
