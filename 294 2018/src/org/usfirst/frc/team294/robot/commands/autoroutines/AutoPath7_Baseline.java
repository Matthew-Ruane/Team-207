package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * If starting on left or right, moves past baseline If starting from middle,
 * moves around pile of cubes and crosses baseline
 */
public class AutoPath7_Baseline extends CommandGroup {

	public AutoPath7_Baseline(StartingPosition startPosition) {
		switch (startPosition) {
		case Middle:
			addSequential(new DriveStraightDistanceProfile(-55, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-60, -90, RobotMap.maxSpeed, RobotMap.maxAcceleration));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-50, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
			break;
		default:
			addSequential(new DriveStraightDistanceProfile(-120, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		}
	}	
}

// Grant's a cool guy.
