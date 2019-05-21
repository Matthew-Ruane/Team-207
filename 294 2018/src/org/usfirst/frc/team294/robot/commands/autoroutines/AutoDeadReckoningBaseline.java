package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A Drive Straight Auto meant to get to the baseline without using any sensors
 * (dead reckoning)
 */
public class AutoDeadReckoningBaseline extends Command {
	private final boolean goForward;
	private static final double MOTOR_POWER = .6, TIME_DELAY = 3;

	/**
	 * Create a new Dead Reckoning Auto
	 * 
	 * @param forward
	 *            true if the auto should move forward, false for backward
	 */
	public AutoDeadReckoningBaseline(boolean forward) {
		requires(Robot.driveTrain);
		this.goForward = forward;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveTrain.driveAtCurve(MOTOR_POWER * (goForward ? 1.0 : -1.0), 0);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timeSinceInitialized() >= TIME_DELAY;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.driveAtCurve(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
