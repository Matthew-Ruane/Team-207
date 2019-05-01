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
import edu.wpi.first.wpilibj.command.Scheduler;

import org.usfirst.frc330.Robot;
import org.usfirst.frc330.util.Logger;

/**
 *
 */
public class KillAll extends BBCommand {

    public KillAll() {

    	requires(Robot.arm);
    	requires(Robot.chassis);
    	requires(Robot.hand);
    	requires(Robot.frills);
    	requires(Robot.climber);
    	requires(Robot.grabber);
    	requires(Robot.lift);
    	
    	this.setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	Logger.getInstance().println("Kill All!", Logger.Severity.WARNING);
    	Scheduler.getInstance().removeAll();
    	Robot.chassis.stopDrive();
    	Robot.arm.stopArm();
    	Robot.hand.stopWrist();
    	Robot.grabber.stopGrabber();
    	Robot.lift.stopLift();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
