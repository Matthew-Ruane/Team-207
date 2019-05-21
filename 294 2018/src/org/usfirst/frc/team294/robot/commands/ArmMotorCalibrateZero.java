package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot; 

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMotorCalibrateZero extends Command {

	/**
	 * Calibrates the current position of the arm as the "zero" position (horizontal arm)
	 */
    public ArmMotorCalibrateZero() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.armMotor); 
    }
    

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.robotPrefs.setArmCalibration( Robot.armMotor.getArmEncRaw(), true);
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
