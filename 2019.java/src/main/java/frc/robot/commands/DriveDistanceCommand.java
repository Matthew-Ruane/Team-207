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

public class DriveDistanceCommand extends Command {

  private Timer timer;
  private boolean holding = false;
  public DriveDistanceCommand() {
    timer = new Timer();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Drivebase.PIDturn.enable();
    Drivebase.PIDleft.enable();
    Drivebase.PIDright.enable();
    Drivebase.setDriveDistance(72);
    Drivebase.PIDturn.setSetpoint(0);
    timer.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Drivebase.pidDrive();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Drivebase.PIDleft.onTarget() || Drivebase.PIDright.onTarget() && holding == false) {
      timer.start();
      holding = true;
      return false;
    }
    else if (Drivebase.PIDleft.onTarget() || Drivebase.PIDright.onTarget() && holding == true && timer.get() >= 0.25) {
      holding = false;
      timer.stop();
      timer.reset();
      return true;
    }
    else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
