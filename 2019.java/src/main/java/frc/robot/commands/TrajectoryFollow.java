/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;

public class TrajectoryFollow extends Command {

  private DistanceFollower dfLeft, dfRight;
  private Drivebase drivebase = Drivebase.getInstance();

  public TrajectoryFollow() {
    requires(drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
/*     Drivebase.zeroLeftEncoder();
    Drivebase.zeroRightEncoder(); */
    double maxVelocityPercentLimit = 1.0;       // Limit max velocity to 0.4 of real max velocity (for safety and to obsereve)
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
      Trajectory.Config.SAMPLES_HIGH, 0.01, Constants.max_velocity_ips*maxVelocityPercentLimit, 
      Constants.max_acceleration_ipsps, Constants.max_jerk_ipspsps);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
