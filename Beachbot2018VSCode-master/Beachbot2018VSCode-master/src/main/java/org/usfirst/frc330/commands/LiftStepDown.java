// RobotBuilder Version: 2.0BB
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc330.commands;
import edu.wpi.first.wpilibj.command.BBCommand;
import org.usfirst.frc330.Robot;
import org.usfirst.frc330.constants.LiftConst;
import org.usfirst.frc330.util.Logger;

/**
 *
 */
public class LiftStepDown extends BBCommand {

	double distance;

    public LiftStepDown(double distance, double timeout) {
        requires(Robot.lift);
        this.distance = distance;
        this.setTimeout(timeout);
    }
    
    public LiftStepDown() {
    	this(LiftConst.stepSize, LiftConst.defaultTimeout);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	double startSetpoint = Robot.lift.getSetpoint();
    	double newSetpoint = startSetpoint - this.distance;
    	
    	Logger.getInstance().println("Starting Lift Setpoint: " + startSetpoint, Logger.Severity.INFO);
    	Logger.getInstance().println("New Lift Setpoint: " + newSetpoint, Logger.Severity.INFO);
    	Robot.lift.setLiftPosition(newSetpoint);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.lift.onLiftTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	Logger.getInstance().println("Ending Lift Position: " + Robot.lift.getPosition(), Logger.Severity.INFO);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	this.end();
    }
}
