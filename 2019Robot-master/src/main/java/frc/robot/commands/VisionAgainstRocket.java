/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HAL;
import frc.robot.subsystems.Turret.TurretPreset;

public class VisionAgainstRocket extends Command {
  double area;
  double xOffset;

  PIDController driveController;
  PIDController turnController;

  double kDrive = -0.3;
  double kTurn = -1.75; //was -0.2 12:36pm 3/23 change -matt
  double max_drive_command = 0.6;
  double min_turn_command = 0.05;

  public VisionAgainstRocket() {
    requires(HAL.limelight);
    requires(HAL.drive);
    HAL.limelight.setPipeline(6);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    area = HAL.limelight.area();
    xOffset = HAL.limelight.xOffset();

    //6.5 IS DESIRED AREA
    double drive = (3.0 - area) * kDrive;

    if(drive > 0 && drive > max_drive_command) {
      drive = max_drive_command;
    } else if (drive < 0 && drive < -max_drive_command) {
      drive = -max_drive_command;
    }

    double turn = (xOffset/27) * kTurn;

    if(turn > 0 && turn < min_turn_command) {
      turn = min_turn_command;
    } else if (turn < 0 && turn > -min_turn_command) {
      turn = -min_turn_command;
    }
    
    HAL.drive.arcade(drive, turn);
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
  }
}
