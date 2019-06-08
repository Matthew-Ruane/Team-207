/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;


public class RotateToAngle extends Command {

  private double heading;
  private Timer timer;
  private int state;
  private int holding = 0;
  private int moving = 1;
  private boolean timerflag = Constants.Off;
  Drivebase drivebase = Drivebase.getInstance();

  public RotateToAngle(double DesiredHeading) {
    heading = DesiredHeading;
    timer = new Timer();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Drivebase.PIDturn.setSetpoint(heading);
    Drivebase.PIDturn.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    drivebase.pidTurn();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Drivebase.PIDturn.onTarget() && state == moving) {
      state = holding;
      return false;
    }
    if (Drivebase.PIDturn.onTarget() && state == holding && timerflag == Constants.Off) {
      timer.start();
      timerflag = Constants.On;
      return false;
    }
    if (Drivebase.PIDturn.onTarget() && state == holding && timer.get() >= 1.0) {
      timer.stop();
      timer.reset();
      timerflag = Constants.Off;
      return true;

    }
    if (!Drivebase.PIDturn.onTarget() && timer.get() > 1.0) {
      timer.reset();
      return false;

    }
    else {
      return false;
  }
}

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivebase.StopDrivetrain();
    drivebase.PIDturn.disable();
    drivebase.PIDturn.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
