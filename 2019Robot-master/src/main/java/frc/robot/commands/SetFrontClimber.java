// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.command.Command;

// import frc.robot.HAL;
// import frc.robot.subsystems.FrontClimber.FrontClimberPreset;


// public class SetFrontClimber extends Command {

//   DoubleSupplier frontClimber;
  
//   private FrontClimberPreset FCPreset;
  
//   public SetFrontClimber(FrontClimberPreset FCPreset) {
//     requires(HAL.frontClimber);
//     this.FCPreset = FCPreset;
//   }

//   public SetFrontClimber(DoubleSupplier frontClimber){
//     this.frontClimber = frontClimber;
//     requires(HAL.frontClimber);
//   }


//   //private FrontClimberPreset frontArm;
//   //DoubleSupplier frontArm
//   // public SetIntakeArm(FrontClimberPreset p) {
//   //   // Use requires() here to declare subsystem dependencies
//   //   // eg. requires(chassis);
//   //   this.frontArm = p;
//   //   requires(HAL.frontClimber);
//   // }

//   // Called just before this Command runs the first time
//     protected void initialize() {

//   }

//   // Called repeatedly when this Command is scheduled to run
//     protected void execute() {
//     HAL.frontClimber.setPosition(FCPreset);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   protected boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   protected void end() {

//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   protected void interrupted() {
//     end();
//   }
// }


package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.HAL;
import frc.robot.subsystems.FrontClimber.FrontClimberPreset;

public class SetFrontClimber extends Command {
  private FrontClimberPreset FCpreset;

  boolean manual;
  DoubleSupplier frontClimber;

  public SetFrontClimber(FrontClimberPreset FCpreset) {
    requires(HAL.frontClimber);
    this.FCpreset = FCpreset;
    manual=false;
  }

  public SetFrontClimber(DoubleSupplier frontClimber) {
    this.frontClimber = frontClimber;
    manual = true;
    requires(HAL.frontClimber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(manual){
    HAL.frontClimber.setSpeed(frontClimber.getAsDouble());
    }
    else{
      HAL.frontClimber.setPosition(FCpreset);
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