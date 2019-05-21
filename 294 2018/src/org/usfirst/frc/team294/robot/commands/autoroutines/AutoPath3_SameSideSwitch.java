package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath3_SameSideSwitch extends CommandGroup {
	
	public AutoPath3_SameSideSwitch(StartingPosition startPosition) {
		int angleMultiplier = 1;
		switch (startPosition) {
		case Left:
			angleMultiplier = 1;
			break;
		case Right: 
			angleMultiplier = -1;
			break;			
		default:
			break;
		}
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentDefault));
//		addParallel(new IntakeSetDeploy(true));
		addSequential(new DriveStraightDistanceProfile(-154, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addParallel(new ArmMoveWithPiston(0, false));
		addSequential(new TurnGyro(-90 * angleMultiplier, TurnGyro.Units.Degrees));
//		addParallel(new IntakeSetDeploy(false));
		addSequential(new WaitCommand(.2));
		addSequential(new DriveStraightDistanceProfile(30, -90 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));  // 24 inches is just shy of the switch (really nice)
		addSequential(new AutoSwitchShoot());
	}
}
