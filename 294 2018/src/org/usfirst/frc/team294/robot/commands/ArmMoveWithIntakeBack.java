package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *Sequence to move the intake out of the way and bring the arm to the switch position
 */
public class ArmMoveWithIntakeBack extends CommandGroup {

	public ArmMoveWithIntakeBack() {
//		addSequential(new IntakeSetDeploy(true));
//		addSequential(new WaitCommand(0.5));
		addSequential(new ArmMoveWithPiston(RobotMap.armScaleLowPos, RobotMap.PistonPositions.Null));
	}
}
