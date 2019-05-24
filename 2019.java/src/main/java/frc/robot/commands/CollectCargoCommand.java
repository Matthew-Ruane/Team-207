package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tray;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;
import frc.robot.Constants;

public class CollectCargoCommand extends Command {
  Elevator elevator = Elevator.getInstance();
  public CollectCargoCommand() {
    requires(elevator);
    isInterruptible();
  }

  @Override
  protected void initialize() {    
    Tray.TalonsRelease();
    Constants.WantHatch = false;
    Tray.UpdateLoadState();
    if (Constants.CARGO_STATE == Constants.CARGO_STATE_UNLOADED) {
        Tray.ExtendTray();
        Tray.IntakeCargo();
        Elevator.SetElevatorPosition(ElevatorPositions.COLLECT, ElevatorModes.CARGO);
    }
  }

  @Override
  protected void execute() {
    Tray.UpdateLoadState();
    }

  @Override
  protected boolean isFinished() {
      if (Constants.CARGO_STATE == Constants.CARGO_STATE_LOADED) {
        Tray.StopIntakeCargo();
        Tray.RetractTray();
        Elevator.SetElevatorPosition(ElevatorPositions.ROCKET_BOTTOM, ElevatorModes.CARGO);
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
