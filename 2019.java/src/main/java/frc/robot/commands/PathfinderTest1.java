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
    
public class PathfinderTest1 extends Command {
  private DistanceFollower dfLeft, dfRight;
  
  private Drivebase drivebase = Drivebase.getInstance();

  public PathfinderTest1() {
    requires(drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {/* 
    Drivebase.zeroLeftEncoder();
    Drivebase.zeroRightEncoder(); */
    Drivebase.setGyroRotation(90.0);   // gyro initial angle (starting robot heading)

    double maxVelocityPercentLimit = 1.0;       // Limit max velocity to 0.4 of real max velocity (for safety and to obsereve)
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
      Trajectory.Config.SAMPLES_HIGH, 0.01, Constants.max_velocity_ips*maxVelocityPercentLimit, 
      Constants.max_acceleration_ipsps, Constants.max_jerk_ipspsps);
    Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0, Pathfinder.d2r(90)),
      new Waypoint(200, 400, Pathfinder.d2r(90)),

    };

    Trajectory trajectory = Pathfinder.generate(points, config);

    // Wheelbase Width
    TankModifier modifier = new TankModifier(trajectory).modify(Constants.wheelbase_in);

    // Create EncoderFollowers for the Trajectories...
    dfLeft = new DistanceFollower(modifier.getLeftTrajectory());
    dfRight = new DistanceFollower(modifier.getRightTrajectory());
    dfLeft.configurePIDVA(0.2, 0.0, 0.0, 1 / Constants.max_velocity_ips, 0);
    dfRight.configurePIDVA(0.2, 0.0, 0.0, 1 / Constants.max_velocity_ips, 0);
    dfLeft.reset();
    dfRight.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
/*     double l = dfLeft.calculate(Drivebase.getLeftDistance());
    double r = dfRight.calculate(Drivebase.getRightDistance()); */

    double gyro_heading = Drivebase.getGyroRotation();    // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(dfLeft.getHeading());  // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = -0.05 * angleDifference;

  //  Drivebase.tank(l+turn, r-turn);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (dfLeft.isFinished()) {
      return true;
    }
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
