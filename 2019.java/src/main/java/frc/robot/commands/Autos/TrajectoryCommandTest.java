/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autos;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;
import frc.robot.subsystems.NavX;

import edu.wpi.first.wpilibj.command.Command;

public class TrajectoryCommandTest extends Command {

private Waypoint[] points = new Waypoint[] {
  new Waypoint (4, 1, Pathfinder.d2r(-45)),
  new Waypoint(2, 2, 0),
  new Waypoint(0, 0, 0)
};

Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60);
Trajectory test_trajectory = Pathfinder.generate(points, config);

TankModifier modifier = new TankModifier(test_trajectory).modify(0.65);

EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());

double l;
double r;
double gyro_heading;
double desired_heading;
double leftoutput;
double rightoutput;
double angleDifference;
double angleDiff;
double turn;


  public TrajectoryCommandTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    left.configureEncoder(Drivebase.leftEncoder.get(), 9800, 0.152);
    left.configurePIDVA(1.0, 0.0, 0.0, .2, 0);
    right.configureEncoder(Drivebase.rightEncoder.get(), 9800, 0.152);
    right.configurePIDVA(1.0, 0.0, 0.0, .2, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    leftoutput = left.calculate(Drivebase.leftEncoder.get());
    rightoutput = right.calculate(Drivebase.rightEncoder.get());

    gyro_heading = NavX.getYaw();
    desired_heading = Pathfinder.r2d(left.getHeading());

    angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    angleDifference = angleDifference % 360.0;
    if (Math.abs(angleDifference) > 180.0) {
      angleDiff = (angleDifference > 0) ? angleDifference - 360 : angleDiff + 360;
    } 

    turn = 0.8*(1.0/80.0)*angleDifference;

    Drivebase.tank(leftoutput + turn, rightoutput - turn);
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
