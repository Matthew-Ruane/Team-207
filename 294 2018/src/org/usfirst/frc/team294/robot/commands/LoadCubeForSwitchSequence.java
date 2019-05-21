package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class LoadCubeForSwitchSequence extends CommandGroup {

	/**
	 * Loads cube to intake rollers only (not to claw).
	 * aka "partial intake"
	 */
    public LoadCubeForSwitchSequence() {

    	addSequential(new ClawSetOpen(false));
 //   	addSequential(new IntakeSetOpen(true));
    	addSequential(new WaitCommand(.75));
 //   	addSequential(new IntakeCube());
    	addSequential(new WaitCommand(0.3));
  //  	addSequential(new IntakeMotorSetToZero());
    }
}
