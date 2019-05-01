/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autocommands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.FollowTrajectory;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;



public class RightFrontRocketBottomHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightFrontRocketBottomHatch() {

    Waypoint[] waypoints = new Waypoint[] {
      new Waypoint(0, 0, 0),
      new Waypoint(5, 0, 0),
    };

    Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 4.5, 2.5, 5.0);

    Trajectory trajectory = Pathfinder.generate(waypoints, config);

  
      addSequential(new FollowTrajectory(trajectory, true));
  


    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
