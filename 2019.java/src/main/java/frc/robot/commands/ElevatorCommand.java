package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;


public class ElevatorCommand extends Command {
  ElevatorPositions position;
  ElevatorModes mode;

  public ElevatorCommand(ElevatorPositions Position, ElevatorModes Mode) {
    position = Position;
    mode = Mode;
  }

  @Override
  protected void initialize() {
    
  }

  @Override
  protected void execute() {
    Elevator.SetElevatorPosition(position, mode);
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
