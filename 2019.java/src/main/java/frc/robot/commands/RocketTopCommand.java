package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPositions;


public class RocketTopCommand extends Command {

  public RocketTopCommand() {
  }

  @Override
  protected void initialize() {
    
  }

  @Override
  protected void execute() {
    Elevator.DesiredPosition = ElevatorPositions.ROCKET_TOP;
    Elevator.SetElevatorPosition();
    SmartDashboard.putNumber("Elevator seeking position", Elevator.getTargetHeight());
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
