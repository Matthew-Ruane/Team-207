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
  private Drivebase drivebase;
  private double distance, lowerbound, upperbound, leftdistance, rightdistance;
  
  public DriveDistanceCommand(double DesiredDistance) {
    distance = DesiredDistance;
    Drivebase.zeroLeftEncoder();
    Drivebase.zeroRightEncoder();
    leftdistance = Drivebase.DistanceInchesToTicks(distance) + Drivebase.leftEncoderZero;
    rightdistance = Drivebase.DistanceInchesToTicks(distance) + Drivebase.rightEncoderZero;
    lowerbound = Drivebase.DistanceInchesToTicks(distance)-Constants.kToleranceDistance;
    upperbound = Drivebase.DistanceInchesToTicks(distance)-Constants.kToleranceDistance;
  }
  @Override
  protected void initialize() {
    Drivebase.EnableVoltComp();
    Drivebase.zeroGyroRotation();
    Drivebase.PIDturn.setSetpoint(Drivebase.getGyroRotation());
    Drivebase.PIDturn.enable();
    state = moving;
  }
  @Override
  protected void execute() {
    Drivebase.motionmagic(leftdistance, rightdistance, Drivebase.getTurnOutput());
  }
  @Override
  protected boolean isFinished() {
    if (Drivebase.getLeftDistance() >= lowerbound && Drivebase.getLeftDistance() <= upperbound && state == moving) {
      state = holding;
      return false;
    }
    if (Drivebase.getLeftDistance() >= lowerbound && Drivebase.getLeftDistance() <= upperbound && state == holding && timerflag == Constants.Off) {
      timer.start();
      timerflag = Constants.On;
      return false;
    }
    if (Drivebase.getLeftDistance() >= lowerbound && Drivebase.getLeftDistance() <= upperbound && state == holding && timer.get() >= 1.0) {
      timer.stop();
      timer.reset();
      timerflag = Constants.Off;
      return true;
    }
    if (!(Drivebase.getLeftDistance() >= lowerbound) || !(Drivebase.getLeftDistance() <= upperbound) && timer.get() > 1.0) {
      timer.reset();
      return false;
    }
    else {
      return false;
    }
  }
  @Override
  protected void end() {
    Drivebase.DisableVoltComp();
    Drivebase.PIDturn.reset();
  }
  @Override
  protected void interrupted() {
    Drivebase.PIDturn.reset();
    Drivebase.DisableVoltComp();
    cancel();
  }
}
