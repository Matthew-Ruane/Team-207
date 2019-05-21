package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ArmMoveWithIntake;
import org.usfirst.frc.team294.robot.commands.ArmMoveWithPiston;
import org.usfirst.frc.team294.robot.commands.ClawSetMotorSpeed;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
//import org.usfirst.frc.team294.robot.commands.IntakeSetDeploy;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath4_OppositeSideSwitchBack extends CommandGroup {

	public AutoPath4_OppositeSideSwitchBack(StartingPosition startPosition) {

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
//		addParallel(new IntakeSetDeploy(true));
		addSequential(new DriveStraightDistanceProfile(-220, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));

		/*
		 * if (Robot.robotPrefs.inBCRLab) { // If on left, then just stop since we are
		 * at the beam if (startPosition == StartingPosition.Left) return;
		 * 
		 * // If on right, then drive short because of beam then stop addSequential(new
		 * DriveStraightDistanceProfile(-152, 90 * angleMultiplier, 150, 150)); //
		 * Actually go 165" but also return; }
		 */

		addParallel(new ArmMoveWithIntake());
		addSequential(new DriveStraightDistanceProfile(-222, 90 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addSequential(new TurnGyro(0 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(65, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(15, 90 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration)); // Touch the Switch wall
		addSequential(new AutoSwitchShoot());

	}
}
