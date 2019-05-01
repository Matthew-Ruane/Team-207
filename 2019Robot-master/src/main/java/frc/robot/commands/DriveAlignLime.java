/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;

public class DriveAlignLime extends Command {

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double xOffset = HAL.limelight.xOffset();
    if(xOffset > 1 || xOffset < -1 ){
      HAL.drive.leftMaster.set(-HAL.limelight.xOffset()/54);
      HAL.drive.rightMaster.set(-HAL.limelight.xOffset()/54);
    } 
    else
    {
      HAL.drive.leftMaster.set(-0.0);
      HAL.drive.rightMaster.set(0.0);
      end();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double xOffset = HAL.limelight.xOffset();
    if(xOffset < 1 && xOffset > -1 ){
      return true;
    }
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
  }
}
