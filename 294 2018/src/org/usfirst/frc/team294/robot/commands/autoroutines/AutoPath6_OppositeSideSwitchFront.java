package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.RobotMap;
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
public class AutoPath6_OppositeSideSwitchFront extends CommandGroup {

	public AutoPath6_OppositeSideSwitchFront(StartingPosition startPosition) {
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
//		addParallel(new IntakeSetDeploy(true));
		addSequential(new DriveStraightDistanceProfile(-53, 0, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		addParallel(new ArmMoveWithPiston(0, false));
		addSequential(new DriveStraightDistanceProfile(-184, 90 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addSequential(new TurnGyro(180, TurnGyro.Units.Degrees));
//		addParallel(new IntakeSetDeploy(false));
		addSequential(new WaitCommand(.2));
		addSequential(new DriveStraightDistanceProfile(47, 180, RobotMap.maxSpeed, RobotMap.maxAcceleration));
		addSequential(new AutoSwitchShoot());
	}
}
