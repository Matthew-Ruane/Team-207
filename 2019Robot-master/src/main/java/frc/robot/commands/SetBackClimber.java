// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.HAL;
// import frc.robot.subsystems.BackClimber.BackClimberPreset;

// public class SetBackClimber extends Command {

//   BackClimberPreset backClimberPreset;

//   public SetBackClimber(BackClimberPreset backClimberPreset) {
//     this.backClimberPreset = backClimberPreset;
//     requires(HAL.backClimber);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     HAL.backClimber.setPosition(backClimberPreset);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//   }
// }

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;
import frc.robot.subsystems.BackClimber.BackClimberPreset;

public class SetBackClimber extends Command {
  private BackClimberPreset BCpreset;

  boolean manual;
  DoubleSupplier backClimber;

  public SetBackClimber(BackClimberPreset BCpreset) {
    requires(HAL.backClimber);
    this.BCpreset = BCpreset;
    manual=false;
  }

  public SetBackClimber(DoubleSupplier backClimber) {
    this.backClimber = backClimber;
    manual = true;
    requires(HAL.backClimber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(manual){
    HAL.backClimber.setSpeed(backClimber.getAsDouble());
    }
    else{
      HAL.backClimber.setPosition(BCpreset);
    }
    //HAL.turret.setPosition(4095);
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