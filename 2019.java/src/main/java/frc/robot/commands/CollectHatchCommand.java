package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tray;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;
import frc.robot.Constants;


public class CollectHatchCommand extends Command {
  private Elevator elevator = Elevator.getInstance();
  private Tray tray = Tray.getInstance();

  public CollectHatchCommand() {
    isInterruptible();
    requires(elevator);
  }

  @Override
  protected void initialize() {
    tray.StopIntakeCargo();
    tray.TalonsRelease();
    tray.ExtendTray();
    Constants.WantHatch = true;
    elevator.SetElevatorPosition(ElevatorPositions.COLLECT, ElevatorModes.HATCH);
  }

  @Override
  protected void execute() {
    tray.TalonsAutoGrab();
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
