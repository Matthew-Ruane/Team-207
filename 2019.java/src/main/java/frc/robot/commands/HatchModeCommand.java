package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPositions;

public class HatchModeCommand extends Command {
  private Elevator elevator = Elevator.getInstance();
  public HatchModeCommand() {
  }

  @Override
  protected void initialize() {
    elevator.SetHatchMode();
    elevator.SetElevatorPosition(elevator.GetElevatorPosition(), elevator.getMode());
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
