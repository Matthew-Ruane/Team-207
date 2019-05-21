package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimbSetPercentPower extends Command {
	private double percentPower;
	private boolean terminate;
	
	/**
	 * Sets the climb motor based on a percent power parameter.
	 * This command immediately terminates (isFinished=true) but
	 * leaves the motor running.
	 * @param percentPower -1 (climb) to +1 (fall)
	 */
    public ClimbSetPercentPower(double percentPower) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.percentPower = percentPower;
    	requires(Robot.climb);
    	terminate = true;
    }
    
	/**
	 * Sets the climb motor based on a percent power parameter.
	 * @param percentPower -1 (climb) to +1 (fall)
     * @param terminate true = immediately terminates (isFinished=true) but
	 * leave the motor running.
	 * false = keep command running, but turn off motor when interrupted.
	 */
    public ClimbSetPercentPower(double percentPower, boolean terminate) {
    	this.terminate = terminate;
    	this.percentPower = percentPower;
    	requires(Robot.climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.climb.setClimbMotors(percentPower);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return terminate;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.climb.setClimbMotors(0);
    }
}
