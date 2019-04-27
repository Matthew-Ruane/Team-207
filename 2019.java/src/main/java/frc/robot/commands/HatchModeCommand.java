package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class HatchModeCommand extends Command {
  public HatchModeCommand() {
  }

  @Override
  protected void initialize() {
    Elevator.SetHatchMode();
    Elevator.SetElevatorPosition();
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
