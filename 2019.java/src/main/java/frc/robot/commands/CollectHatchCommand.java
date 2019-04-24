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


public class CollectHatchCommand extends Command {
  private static Tray TraySubsystem;
  public CollectHatchCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(TraySubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Elevator.DesiredPosition = ElevatorPositions.COLLECT;
    Elevator.Mode = ElevatorModes.HATCH;
    Tray.StopIntakeCargo();
    Tray.TalonsRelease();
    Tray.ExtendTray();
    Tray.WantHatch = true;
    Elevator.SetElevatorPosition();
    Tray.TalonsAutoGrab();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Tray.WantHatch == false) {
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
