package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPositions;


public class RocketBottomCommand extends Command {

  public RocketBottomCommand() {
  }

  @Override
  protected void initialize() {
    
  }

  @Override
  protected void execute() {
    Elevator.DesiredPosition = ElevatorPositions.ROCKET_BOTTOM;
    Elevator.SetElevatorPosition();
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
