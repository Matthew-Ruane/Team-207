package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *Sequence to move the intake out of the way and bring the arm to the switch position
 *Still called in Auto routines even though no longer an Intake
 */
public class ArmMoveWithIntake extends CommandGroup {

	/**
	 * Move arm and intake to position to score on switch.
	 * 
	 * Details:  Deploys intake, wait 0.25 secs to deploy, move arm to switch position, wait 1 sec,
	 * and then raise intake (so we can butt up against the switch)
	 */
	public ArmMoveWithIntake() {
//		addSequential(new IntakeSetDeploy(true));
		addSequential(new WaitCommand(0.25));		// TODO This delay may not be necessary since there is no Intake  - remove and test
		addParallel(new ArmMoveWithPiston(RobotMap.armSwitchPosHigh, false));
		addSequential(new WaitCommand(1.0));
//		addSequential(new IntakeSetDeploy(false));
	}
}
