// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.Autos;

// import frc.robot.ninjaLib.PathLoader;
// import frc.robot.subsystems.*;
// import frc.robot.commands.*;
// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.command.CommandGroup;
// import jaci.pathfinder.Pathfinder;
// import jaci.pathfinder.Trajectory;
// import jaci.pathfinder.Waypoint;



// public class LeftSideRocketHatch extends CommandGroup {
//   Waypoint[] habToRocket = new Waypoint[] {
//     new Waypoint(0, 0, 0),
//     new Waypoint(10, 6, 30)
//     };

//   Waypoint[] rocketToFeed = new Waypoint[]{
//     new Waypoint(10, 6, 30),
//     new Waypoint(9, -3, 0)
//   };

//   Waypoint[] feedToRocket = new Waypoint[]{
//     new Waypoint(9, -3, 0),
//     new Waypoint(7, 8, 0),
//     new Waypoint(10, 11, -30),
//     new Waypoint(11, 10, -30)
//   };

// Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
//     0.02, 	//delta time
//     4.5,		//max velocity in ft/s for the motion profile
//     2.5,		//max acceleration in ft/s/s for the motion profile
//     5.0);	//max jerk in ft/s/s/s for the motion profile


// Trajectory trajectory = Pathfinder.generate(habToRocket, config);
// Trajectory trajectory2 = Pathfinder.generate(rocketToFeed, config);
// Trajectory trajectory3 = Pathfinder.generate(feedToRocket, config);

// public LeftSideRocketHatch(boolean backwards) {
//   addSequential(new FollowTrajectory(trajectory, backwards));
//   addSequential(new FollowTrajectory(trajectory2, !backwards));
//   addParallel(new SetTurret(Turret.TurretPreset.BACK));
//   addSequential(new FollowTrajectory(trajectory3, backwards));
//   addParallel(new SetElevatorShoulderWrist(Wrist.WristPreset.ROCKET_LOW_HATCH,Shoulder.ShoulderPreset.ROCKET_LOW_HATCH,Elevator.ElevatorPreset.ROCKET_LOW_HATCH), 2);
// }
// }
