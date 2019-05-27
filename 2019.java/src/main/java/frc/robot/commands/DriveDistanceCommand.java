/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;

public class DriveDistanceCommand extends Command {

  private Timer timer;
  private int state;
  private int holding = 0;
  private int moving = 1;
  private boolean timerflag = Constants.Off;
  private double distance, LeftDistanceTarget, RightDistanceTarget, distanceTraveled, heading;
  
  public DriveDistanceCommand(double DesiredDistance) {
    distance = DesiredDistance;
    Constants.DesiredDistance = distance;
    Drivebase.zeroLeftEncoder();
    Drivebase.zeroRightEncoder();
    Constants.LeftDistanceTarget = Drivebase.DistanceInchesToTicks(distance) + Drivebase.leftEncoderZero;
    Constants.RightDistanceTarget = Drivebase.DistanceInchesToTicks(distance) + Drivebase.rightEncoderZero;
  }
  @Override
  protected void initialize() {
    Drivebase.zeroGyroRotation();
    heading = Drivebase.getGyroRotation();
    Drivebase.PIDturn.setSetpoint(heading);
    SmartDashboard.putNumber("driveheading", heading);
    Drivebase.PIDturn.enable();
    Constants.TurnOutput = Drivebase.getTurnOutput();
    state = moving;
    Drivebase.motionmagic(LeftDistanceTarget, RightDistanceTarget, Constants.TurnOutput);
  }
  @Override
  protected void execute() {
    Constants.TurnOutput = Drivebase.getTurnOutput();
  }
  @Override
  protected boolean isFinished() {
    distanceTraveled = Drivebase.getLeftDistance();
    if (Drivebase.onTargetDistance(LeftDistanceTarget, distanceTraveled) && state == moving) {
      state = holding;
      return false;
    }
    if (Drivebase.onTargetDistance(LeftDistanceTarget, distanceTraveled) && state == holding && timerflag == Constants.Off) {
      timer.start();
      timerflag = Constants.On;
      return false;
    }
    if (Drivebase.onTargetDistance(LeftDistanceTarget, distanceTraveled) && state == holding && timer.get() >= 1.0) {
      timer.stop();
      timer.reset();
      timerflag = Constants.Off;
      return true;
    }
    if (!Drivebase.onTargetDistance(LeftDistanceTarget, distanceTraveled) && timer.get() > 1.0) {
      timer.reset();
      return false;
    }
    else {
      return false;
    }
  }
  @Override
  protected void end() {
    Drivebase.PIDturn.reset();
  }
  @Override
  protected void interrupted() {
    Drivebase.PIDturn.reset();
    cancel();
  }
}
