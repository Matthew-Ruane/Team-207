package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class ClimbPreparation extends CommandGroup {

	/**
	 * Preapres robot to climb:
	 * Lowers intake, closes claw, raises arm, and deploys climbing hook
	 */
    public ClimbPreparation() {
    	addParallel(new ClawSetOpen(false));
//    	addSequential(new IntakeSetDeploy(true));
    	addSequential(new WaitCommand(0.25));
    	addParallel(new ArmMoveWithPiston(RobotMap.armClimbPos, false));
    	addSequential(new ClimbPistonDeploy(true));
    }
}
