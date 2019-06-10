package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tray;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;
import frc.robot.Constants;

public class CollectCargoCommand extends Command {
  private Elevator elevator = Elevator.getInstance();
  private Tray tray = Tray.getInstance();

  public CollectCargoCommand() {
    requires(elevator);
    isInterruptible();
  }

  @Override
  protected void initialize() {    
    tray.TalonsRelease();
    Constants.WantHatch = false;
    tray.UpdateLoadState();
    if (Constants.CARGO_STATE == Constants.CARGO_STATE_UNLOADED) {
        tray.ExtendTray();
        tray.IntakeCargo();
        elevator.SetElevatorPosition(ElevatorPositions.COLLECT, ElevatorModes.CARGO);
    }
  }

  @Override
  protected void execute() {
    tray.UpdateLoadState();
    }

  @Override
  protected boolean isFinished() {
      if (Constants.CARGO_STATE == Constants.CARGO_STATE_LOADED) {
        tray.StopIntakeCargo();
        tray.RetractTray();
        elevator.SetElevatorPosition(ElevatorPositions.ROCKET_BOTTOM, ElevatorModes.CARGO);
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
    cancel();
    
  }
}
