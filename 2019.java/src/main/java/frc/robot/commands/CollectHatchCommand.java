package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tray;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;
import frc.robot.Constants;


public class CollectHatchCommand extends Command {
  private Elevator elevator = Elevator.getInstance();
  public CollectHatchCommand() {
    isInterruptible();
    requires(elevator);
  }

  @Override
  protected void initialize() {
    Tray.StopIntakeCargo();
    Tray.TalonsRelease();
    Tray.ExtendTray();
    Constants.WantHatch = true;
    Elevator.SetElevatorPosition(ElevatorPositions.COLLECT, ElevatorModes.HATCH);
  }

  @Override
  protected void execute() {
    Tray.TalonsAutoGrab();
  }

  @Override
  protected boolean isFinished() {
    if (Constants.WantHatch == false) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
    Constants.WantHatch = false;
    cancel();
  }
}
