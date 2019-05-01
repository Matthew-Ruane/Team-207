package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import java.util.function.DoubleSupplier;


import frc.robot.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoldTurret extends Command {
  double holdPos;
  DoubleSupplier nudgeJoystick;
  public HoldTurret(DoubleSupplier nudgeJoystick) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(HAL.turret);
    this.nudgeJoystick = nudgeJoystick;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    holdPos = HAL.turret.getCurrentPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double nudge = nudgeJoystick.getAsDouble();
    if (Math.abs(nudge) > 0.05) {
      HAL.turret.setSpeed(nudge);
      holdPos = HAL.turret.getCurrentPosition();
    } else {
      HAL.turret.setPosition(holdPos);
    }
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
