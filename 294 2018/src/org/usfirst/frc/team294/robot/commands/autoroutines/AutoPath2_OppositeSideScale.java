package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ArmMoveWithIntake;
import org.usfirst.frc.team294.robot.commands.ArmMoveWithIntakeBack;
import org.usfirst.frc.team294.robot.commands.ArmMoveWithPiston;
import org.usfirst.frc.team294.robot.commands.ArmPistonSmartExtendInDestZone;
import org.usfirst.frc.team294.robot.commands.ClawSetMotorSpeed;
import org.usfirst.frc.team294.robot.commands.CubeShootOut;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.LoadCubeSequence;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath2_OppositeSideScale extends CommandGroup {

	public AutoPath2_OppositeSideScale(StartingPosition startPosition) {
		int angleMultiplier = 1;

		switch (startPosition) {
		case Left:
			angleMultiplier = 1;
			break;
		case Right:
			angleMultiplier = -1;
			break;
		}
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentDefault));
		addSequential(new WaitCommand(0.1));
		addParallel(new ArmMoveWithIntake());
//		addSequential(new IntakeSetOpen(true));
		addSequential(new DriveStraightDistanceProfile(-220, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(-220, 90 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration)); // was -220
		addParallel(new ArmMoveWithIntakeBack());
		addSequential(new TurnGyro(165 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(40, 165 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration)); // was 36
		addSequential(new ArmPistonSmartExtendInDestZone(RobotMap.armScaleLowPos));
		addSequential(new WaitCommand(0.5));
		//addSequential(new WaitCommand(0.5));
		addSequential(new CubeShootOut());
//		addSequential(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));//true)); // enforce the arm being up before shooting
		addSequential(new ArmMoveWithPiston(90, RobotMap.PistonPositions.Null));//true)); // move arm to +90, since the arm will stay there when disabled
		
		addParallel(new TurnGyro(0, TurnGyro.Units.Degrees));
		addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos, false));
//		addSequential(new LoadCubeSequence());

		// addParallel(new ClawSetMotorSpeed(-0.40));
		// addSequential(new DriveStraightDistanceProfile(-200, 0, 120, 150));
		// addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		// addSequential(new DriveStraightDistanceProfile(-180, 90 * angleMultiplier,
		// 120, 150));
		// addSequential(new TurnGyro(-45 * angleMultiplier, TurnGyro.Units.Degrees));
		// addSequential(new DriveStraightDistanceProfile(-50, -45 * angleMultiplier));
		// // Backwards dunk cube
		// addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
		// addSequential(new DriveStraightDistanceProfile(50, 0)); // Drive back to
		// "normal" angle
		// // Do second cube?

	}
}
