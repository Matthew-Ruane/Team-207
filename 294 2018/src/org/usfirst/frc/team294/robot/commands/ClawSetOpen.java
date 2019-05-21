package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClawSetOpen extends Command {

	boolean open;
	
	/**
	 * Set the state of the claw
	 * @param open true = claw open, false = claw closed
	 */
    public ClawSetOpen(boolean open) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.claw);
    	this.open = open;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (open) Robot.claw.openClaw();
    	else Robot.claw.closeClaw();
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
