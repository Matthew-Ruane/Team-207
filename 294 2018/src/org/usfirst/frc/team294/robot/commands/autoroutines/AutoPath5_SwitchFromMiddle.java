package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmPositions;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;
import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath5_SwitchFromMiddle extends CommandGroup {
	int angleMultiplier;

	/**
	 * Robot starts in the middle and puts cube on switch.
	 * 
	 * @param goLeft
	 *            true = put cube on left plate, false = put cube on right plate
	 */
	public AutoPath5_SwitchFromMiddle(boolean goLeft) {
		if (goLeft) {
			angleMultiplier = -1;
		} else {
			angleMultiplier = 1;
		}
		
		// Go to drive config
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentInFully));
//		addParallel(new IntakeSetDeploy(false));
		addSequential(new WaitCommand(0.1)); 
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentDefault));
		addParallel(new ArmMoveWithIntake());
		
		// Go forward to switch
//		addSequential(new DriveStraightDistanceProfile(10, 0, 100, 100));
		if (goLeft) {
			addSequential(new DriveStraightDistanceProfile(118, 26 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration)); // acceleration factor was 7, distance was 110
			
			
		} else { // going right
			addSequential(new DriveStraightDistanceProfile(108, 19 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));  // acceleration factor was 70, distance was 100	
		}
		
		// Shoot out first cube
		addSequential(new AutoSwitchShoot());
		
		// Back away from switch to front of cube pile
		addSequential(new DriveStraightDistanceProfile(-105, 23 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));  // acceleration factor was 70 
		
		// Start intake and drive forward to grab 2nd cube
		addParallel(new LoadCubeSequence(ArmPositions.Intake, PistonPositions.Extended, false));
//		addParallel(new LoadCubeSequenceWithIntakeOpenAuto());
		
		addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
		//addSequential(new TurnGyro()); 
		//			addParallel(new LoadCubeSequenceWithIntakeOpenAuto());	// open claw to allow for more error on position
		//addParallel(new LoadCubeSequence());		// TODO: This was redundant, but need to check!!!!
		//addParallel(new IntakeSetOpen(true));		// This was needed to force orpen intake,  it closed before moving forward
		addSequential(new DriveStraightDistanceProfile(60, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		
//		addSequential(new DriveStraightDistanceProfile(-10, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		
		// If we have the cube in the intake in diamond shape (not in claw), then try rotating the cube
//		addSequential(new AutoRotateAndIntakeDiamondCube());

		// If we don't have the cube in the claw, then stop (well, wait 15 sec to end of auto mode)
//    	addSequential(new ConditionalCommand(new WaitCommand(15)) {
//			protected boolean condition() {
//				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
//				+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
//				return (!Robot.claw.getPhotoSwitch());
//			}
//		});

		// Move arm up to switch position and back away from pile
		addParallel(new ArmMoveWithIntake());
		addSequential(new DriveStraightDistanceProfile(-60, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration)); // distance was -60, but changed because we back up 10 inches earlier
		
		// Turn towards switch and drive to switch
		addSequential(new TurnGyro(22 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(108, 25 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));  // acceleration factor was 70 
		
		// Score 2nd cube at switch
		addSequential(new AutoSwitchShoot());

		addSequential(new DriveStraightDistanceProfile (-8, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addParallel(new TurnGyro (-90 * angleMultiplier, TurnGyro.Units.Degrees));
//		addSequential(new LoadCubeSequence());
		addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos, false));//true)); // enforce the arm being up before shooting

	}
}
