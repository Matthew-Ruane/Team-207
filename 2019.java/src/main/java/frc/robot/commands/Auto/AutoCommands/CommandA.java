/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.Auto.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.commands.Auto.AutoModeBase;
import frc.robot.commands.Auto.AutoModeEndedException;
import frc.robot.commands.Auto.Actions.FollowPathAction;
import frc.robot.commands.Auto.Actions.SeriesAction;
import frc.robot.subsystems.Drivebase;
import frc.utility.PurePursuit.Path;
import frc.utility.PurePursuit.Path.Waypoint;
import frc.utility.PurePursuit.Rotation2d;
import frc.utility.PurePursuit.Translation2d;
import frc.robot.subsystems.Drivebase;

public class CommandA extends Command {

  Path path;
  public CommandA() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting 2 ball auto mode...");
    List<Waypoint> first_path = new ArrayList<>();
    first_path.add(new Waypoint(new Translation2d(0, 0), 120.0));
    first_path.add(new Waypoint(new Translation2d(24, 0), 120.0));

    path = new Path(first_path);
    Drivebase.followPath(path, false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Drivebase.updatePathFollower();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
