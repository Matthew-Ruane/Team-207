package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class ClimbMotorSequence extends CommandGroup {

	/**
	 * Winches the robot up and then holds it up.
	 * This command assumes that the robot has already clamped onto the bar.
	 */
	public ClimbMotorSequence() {
    	addParallel(new ArmMoveWithPiston(RobotMap.armIntakePos, false));
    	addParallel(new ClimbPistonDeploy(false)); //setting up 
    	addSequential(new WaitCommand(0.5));
    	addParallel(new ClimbSetPercentPower(RobotMap.climbPercent));
    	addSequential(new WaitCommand(4.0));			//  test this time or add sensor when over 12 inches
    	addSequential(new ClimbSetPercentPower(RobotMap.climbHoldRobot));
    	addSequential(new WaitCommand(26.0));
    	addSequential(new ClimbSetPercentPower(0.0));
    }
}
