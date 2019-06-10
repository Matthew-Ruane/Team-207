/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto.AutoCommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.subsystems.Drivebase;
import frc.utility.PurePursuit.Path;
import frc.utility.PurePursuit.Path.Waypoint;
import frc.utility.PurePursuit.Rotation2d;
import frc.utility.PurePursuit.Translation2d;
import frc.utility.PurePursuit.RigidTransform2d;
import frc.utility.PurePursuit.Kinematics;
import frc.robot.RobotState;
import frc.robot.Auto.AutoPaths;
import edu.wpi.first.wpilibj.Timer;

public class SetPath extends Command {

  Path path;

  Rotation2d gyro_angle;
  RigidTransform2d odometry;
  RigidTransform2d.Delta velocity;
  RobotState robotstate = RobotState.getInstance();
  Drivebase drivebase = Drivebase.getInstance();


  public SetPath(Path path) {
    this.path = path;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drivebase.normalizeEncoders();
    drivebase.UpShift();
    drivebase.followPath(path, false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    drivebase.updateRobotState();
    drivebase.updatePathFollower();
    SmartDashboard.putNumber("path", path.getRemainingLength());
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return drivebase.isFinishedPath();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivebase.StopDrivetrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
