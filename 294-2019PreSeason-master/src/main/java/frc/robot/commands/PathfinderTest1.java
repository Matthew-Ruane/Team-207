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
import frc.robot.Robot;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;
    
public class PathfinderTest1 extends Command {
  private DistanceFollower dfLeft, dfRight;

  public PathfinderTest1() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.log.writeLogEcho("Pathfinder", "initialize", "");
    Robot.drivetrain.zeroLeftEncoder();
    Robot.drivetrain.zeroRightEncoder();
    Robot.drivetrain.setGyroRotation(90.0);   // gyro initial angle (starting robot heading)
    Robot.drivetrain.setVoltageCompensation(true);

    double maxVelocityPercentLimit = 1.0;       // Limit max velocity to 0.4 of real max velocity (for safety and to obsereve)
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
      Trajectory.Config.SAMPLES_HIGH, 0.01, RobotMap.max_velocity_ips*maxVelocityPercentLimit, 
      RobotMap.max_acceleration_ipsps, RobotMap.max_jerk_ipspsps);
    Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0, Pathfinder.d2r(90)),
      // new Waypoint(0, 200, Pathfinder.d2r(90))
      new Waypoint(24, 54, Pathfinder.d2r(60)),
      new Waypoint(36, 108, Pathfinder.d2r(90)),
      new Waypoint(24, 162, Pathfinder.d2r(120)),
      new Waypoint(0, 216, Pathfinder.d2r(90))
    };

    Trajectory trajectory = Pathfinder.generate(points, config);

    // Save main trajectory for reference
    File saveFile = new File("/home/lvuser/trajectory.csv");
    Pathfinder.writeToCSV(saveFile, trajectory);

    // Wheelbase Width
    TankModifier modifier = new TankModifier(trajectory).modify(RobotMap.wheelbase_in);

    // Create EncoderFollowers for the Trajectories...
    dfLeft = new DistanceFollower(modifier.getLeftTrajectory());
    dfRight = new DistanceFollower(modifier.getRightTrajectory());
    // dfLeft.configureEncoder(Robot.drivetrain.getLeftEncoderTicks(), RobotMap.encoderTicksPerRevolution, RobotMap.wheel_diameter_m);
    // dfRight.configureEncoder(Robot.drivetrain.getRightEncoderTicks(), RobotMap.encoderTicksPerRevolution, RobotMap.wheel_diameter_m);
    dfLeft.configurePIDVA(0.02, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0025);
    dfRight.configurePIDVA(0.02, 0.0, 0.0, 1 / RobotMap.max_velocity_ips, 0.0025);
    dfLeft.reset();
    dfRight.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double l = dfLeft.calculate(Robot.drivetrain.getLeftDistance());
    double r = dfRight.calculate(Robot.drivetrain.getRightDistance());

    double gyro_heading = Robot.drivetrain.getGyroRotation();    // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(dfLeft.getHeading());  // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = -0.016 * angleDifference;
    
    Robot.drivetrain.setLeftMotors(l + turn);
    Robot.drivetrain.setRightMotors(r - turn);

    Trajectory.Segment seg = dfLeft.getSegment();
    Robot.log.writeLog("Pathfinder", "execute", "left power," + l + ",right power," + r + ",turn power," + turn +
                       ",left distance," + Robot.drivetrain.getLeftDistance() + ",right distance," + Robot.drivetrain.getRightDistance() +
                       ",heading," + gyro_heading + ",isFinished," + dfLeft.isFinished() + 
                       ",segPos," + seg.position + ",segVel," + seg.velocity + ",segAccel," + seg.acceleration + 
                       ",segJerk," + seg.jerk + ",segHeading," + desired_heading + ",segdt," + seg.dt);
    SmartDashboard.putNumber("Pathfinder left position", seg.position);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (dfLeft.isFinished()) {
      Robot.drivetrain.setVoltageCompensation(false);
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
