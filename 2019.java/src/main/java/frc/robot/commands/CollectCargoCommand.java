/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tray;
import frc.robot.subsystems.Elevator.ElevatorModes;
import frc.robot.subsystems.Elevator.ElevatorPositions;

public class CollectCargoCommand extends Command {
  public CollectCargoCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      Tray.TalonsRelease();
      Tray.WantHatch = false;
      Tray.UpdateLoadState();
      if (Tray.CARGO_STATE == Tray.CARGO_STATE_UNLOADED) {
          Tray.ExtendTray();
          Elevator.DesiredPosition = ElevatorPositions.COLLECT;
          Elevator.Mode = ElevatorModes.CARGO;
          Tray.IntakeCargo();
          Elevator.SetElevatorPosition();
      }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      Tray.UpdateLoadState();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
      if (Tray.CARGO_STATE == Tray.CARGO_STATE_LOADED) {
        Tray.StopIntakeCargo();
        Tray.RetractTray();
        Elevator.DesiredPosition = ElevatorPositions.ROCKET_BOTTOM;
        return true;
      }
      else {
        return false;
      }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
