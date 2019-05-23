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
import frc.utility.PIDGains;
import frc.robot.Constants;

public class DriveDistanceCommand extends Command {

  private Timer timer;
  private int state;
  private int holding = 0;
  private int moving = 1;
  private boolean timerflag = Constants.Off;
  private Drivebase drivebase;
  private double distance, heading, tolerance, maxOutput, maxOutputStep, maxOutputMax, leftdistance, rightdistance;
  private PIDGains gains;
  
  public DriveDistanceCommand(double DesiredDistance, PIDGains gains) {
    Drivebase.zeroLeftEncoder();
    Drivebase.zeroRightEncoder();
    /* leftdistance = DesiredDistance + Drivebase.leftEncoderZero;
    rightdistance = DesiredDistance + Drivebase.rightEncoderZero; */
    leftdistance = 500000;
    rightdistance = 500000;
    this.gains = gains;
    maxOutput = 0;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Drivebase.EnableVoltComp();
    Drivebase.zeroGyroRotation();
    Drivebase.PIDturn.setSetpoint(Drivebase.getGyroRotation());

    state = moving;
    maxOutput = gains.getMinStartOutput();
    maxOutputMax = gains.getMaxOutput();
    maxOutputStep = gains.getMaxOutputStep();
    Drivebase.PIDturn.enable();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Drivebase.motionmagic(leftdistance, rightdistance, Drivebase.getTurnOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    /* if (Drivebase.PIDleft.onTarget() || Drivebase.PIDright.onTarget() && state == moving) {
      state = holding;
      return false;
    }
    if (Drivebase.PIDleft.onTarget() || Drivebase.PIDright.onTarget() && state == holding && timerflag == Constants.Off) {
      timer.start();
      timerflag = Constants.On;
      return false;
    }
    if (Drivebase.PIDleft.onTarget() || Drivebase.PIDright.onTarget() && state == holding && timer.get() >= 1.0) {
      timer.stop();
      timer.reset();
      timerflag = Constants.Off;
      return true;
    }
    if (!Drivebase.PIDleft.onTarget() || !Drivebase.PIDright.onTarget() && timer.get() > 1.0) {
      timer.reset();
      return false;
    }
    else {
      return false;
    } 
    if (Drivebase.PIDturn.onTarget() && Drivebase.PIDleft.onTarget() || Drivebase.PIDright.onTarget()) {
      return true;
    }
    else {
      return false;
    } */
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Drivebase.DisableVoltComp();
    Drivebase.PIDturn.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Drivebase.PIDturn.reset();
    Drivebase.DisableVoltComp();
    cancel();
  }
}
