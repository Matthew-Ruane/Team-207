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

  public static Ultrasonic mRangefinder;
  public static double kDistance;
  public static SendableBuilder RangefinderDisplay;

  public Rangefinder() {
    mRangefinder = new Ultrasonic(RobotMap.mUltrasonic_Ping_ID, RobotMap.mUltrasonic_Echo_ID, Unit.kInches);
    mRangefinder.setEnabled(true);
    kDistance = mRangefinder.getRangeInches();
  }

  public static double getDistance() {
    return kDistance;
  }

  @Override
  public void initDefaultCommand() {
  }
}
