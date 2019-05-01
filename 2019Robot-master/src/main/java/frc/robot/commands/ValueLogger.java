/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import frc.robot.HAL;

public class ValueLogger extends Command {
  public ValueLogger() {
    requires(HAL.wrist);
    requires(HAL.shoulder);
    requires(HAL.frontClimber);
    requires(HAL.ballFeed);
    requires(HAL.turret);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
  
;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    NetworkTable pathTable = NetworkTable.getTable("SmartDashboard");
    pathTable.putDouble("wrist", HAL.wrist.getCurrentPosition());
    pathTable.putDouble("shoudler", HAL.shoulder.getCurrentPosition());
    pathTable.putDouble("intake arm", HAL.frontClimber.getCurrentPosition());
    pathTable.putDouble("feed", HAL.wrist.getCurrentPosition());
    pathTable.putDouble("turret", HAL.turret.getCurrentPosition());
    pathTable.putDouble("elevator", HAL.elevator.getCurrentPosition());
    pathTable.putDouble("camtran", HAL.limelight.camtran());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
