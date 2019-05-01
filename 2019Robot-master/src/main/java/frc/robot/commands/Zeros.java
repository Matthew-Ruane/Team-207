/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;

public class Zeros extends Command {
  String system;
  public Zeros(String a) {
    // Use requires() here to declare subsystem dependencies
    system = a;
    requires(HAL.shoulder);
    requires(HAL.turret);
    requires(HAL.wrist);
    requires(HAL.elevator);
    requires(HAL.frontClimber);
    requires(HAL.backClimber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { 
  if(system.equals("shoulder"))
  HAL.shoulder.setZero();
  else if(system.equals("turret"))
  HAL.turret.setZero();
  else if(system.equals("wrist"))
  HAL.wrist.setZero();
  else if(system.equals("elevator"))
  HAL.elevator.setZero();
  else if(system.equals("frontClimber"))
  HAL.frontClimber.setZero();
  else{
  HAL.backClimber.setZero();
  }

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  end();
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
