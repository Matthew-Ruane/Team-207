/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.utility.PurePursuit.RigidTransform2d;
import frc.utility.PurePursuit.Kinematics;
import frc.robot.subsystems.Drivebase;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class CommandA extends Command {

  Path path;

  double left_encoder_prev_distance_, right_encoder_prev_distance_, left_distance, right_distance, time;
  Rotation2d gyro_angle;
  RigidTransform2d odometry;
  RigidTransform2d.Delta velocity;
  RobotState robotstate = RobotState.getInstance();


  public CommandA() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    left_encoder_prev_distance_ = Drivebase.getLeftDistanceInches();
    right_encoder_prev_distance_ = Drivebase.getRightDistanceInches();
    Drivebase.resetEncoders();
    Drivebase.DownShift();
    
    List<Waypoint> first_path = new ArrayList<>();
    first_path.add(new Waypoint(new Translation2d(0, 0), 100.0));
    first_path.add(new Waypoint(new Translation2d(72, 0), 100.0));
    first_path.add(new Waypoint(new Translation2d(72, 72), 100.0));

    path = new Path(first_path);
    Drivebase.followPath(path, false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("path", path.getRemainingLength());

    time = Timer.getFPGATimestamp();
    left_distance = Drivebase.getLeftDistanceInches();
    right_distance = Drivebase.getRightDistanceInches();
    gyro_angle = Drivebase.getGyroAngle();
    odometry = robotstate.generateOdometryFromSensors(left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
    velocity = Kinematics.forwardKinematics(Drivebase.getLeftVelocityInchesPerSec(), Drivebase.getRightVelocityInchesPerSec());
    robotstate.addObservations(time, odometry, velocity);
    left_encoder_prev_distance_ = left_distance;
    right_encoder_prev_distance_ = right_distance;
    
    Drivebase.updatePathFollower();
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Drivebase.isFinishedPath();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Drivebase.StopDrivetrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
