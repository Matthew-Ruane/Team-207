package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Shift extends Command {

	private boolean high;

	/**
	 * changes from low to high gear on drive
	 * 
	 * @param high
	 *            true is high gear, false is low gear
	 */
	public Shift(boolean high) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.shifter);
		this.high = high;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (high) { // true is high gear, false is low gear
			Robot.shifter.setShift(true);
		} else {
			Robot.shifter.setShift(false);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
