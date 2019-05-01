/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;

public class SetElevator extends Command {
  DoubleSupplier nudge;
  boolean manual;
  public SetElevator() {
    requires(HAL.elevator);
    manual = false;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  public SetElevator(DoubleSupplier nudge)
  {
    manual = true;
    this.nudge = nudge;
    requires(HAL.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //HAL.elevator.setPosition(40);
  }

  // Called repeatedly when this Command is scheduled to run
  
  protected void execute() {
    //HAL.elevator.setPosition(HAL.elevator.getCurrentPosition() + nudge.getAsDouble());
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
