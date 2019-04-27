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
        Elevator.DesiredPosition = ElevatorPositions.COLLECT;
        Elevator.Mode = ElevatorModes.CARGO;
        Tray.IntakeCargo();
        Elevator.SetElevatorPosition();
    }
  }

  @Override
  protected void execute() {
    }

  @Override
  protected boolean isFinished() {
      if (Constants.CARGO_STATE == Constants.CARGO_STATE_LOADED) {
        Tray.StopIntakeCargo();
        Tray.RetractTray();
        Elevator.DesiredPosition = ElevatorPositions.ROCKET_BOTTOM;
        Elevator.SetElevatorPosition();
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
