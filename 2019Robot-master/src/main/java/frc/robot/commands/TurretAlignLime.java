/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;
import frc.robot.subsystems.LimelightSubsystem;

public class TurretAlignLime extends Command {
  private static final double kP = 1;
  int height; 
  public TurretAlignLime(int height) {
    this.height = height;
    requires(HAL.turret);
  }
  
  @Override
  protected void initialize() {
  }
// MORE NEGATIVE MOVES IT TO THE LEFT
// MORE POSITIVE MOVES IT TO THE RIGHT
  @Override
  protected void execute() {
    double power;
    double xOffset = HAL.limelight.xOffset();
    if(height == 1){
    power = kP * ((xOffset - 8.5)/27); //was 8.6
    }
    else{
    power = kP * ((xOffset - 12.8)/27); //was 13
    }
    double nopower = 0;
    if (HAL.limelight.hasTargets())
    {
      HAL.turret.setSpeed(power);
    }
    else
    {
      HAL.turret.setSpeed(nopower);
    }

  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //HAL.limelight.setPipeline(8);
  }
}
