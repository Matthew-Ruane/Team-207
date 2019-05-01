package frc.robot.commands;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.HAL;
import frc.robot.subsystems.Turret.TurretPreset;

public class SetTurret extends Command {
  private TurretPreset Tpreset;

  DoubleSupplier turret;

  public SetTurret(TurretPreset Tpreset) {
    requires(HAL.turret);
    this.Tpreset = Tpreset;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    HAL.turret.setPosition(Tpreset);
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