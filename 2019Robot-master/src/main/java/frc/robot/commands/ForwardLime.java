/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;

public class ForwardLime extends Command {
  @Override
  protected void initialize() {
  }

  
  @Override
  protected void execute() {
    if (HAL.limelight.area() <= 1.5)
    {
      HAL.drive.leftMaster.set(-0.2);
      HAL.drive.rightMaster.set(0.2);
    }
    else
    {
      HAL.drive.leftMaster.set(-0.0);
      HAL.drive.rightMaster.set(0.0);
      end();
    }

  }

  
  @Override
  protected boolean isFinished() {
    if(HAL.limelight.area() > 1.5)
    {
      return true;
    }
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
