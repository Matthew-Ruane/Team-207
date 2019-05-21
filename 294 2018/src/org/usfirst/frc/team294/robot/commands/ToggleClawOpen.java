package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ToggleClawOpen extends Command {

    public ToggleClawOpen() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.claw);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.claw.isClawOpen())
    	{
    		Robot.claw.closeClaw();
    	}
    	
    	else 
    	{
    		Robot.claw.openClaw();
    	}
//    	Robot.claw.setClawMotorPercent(RobotMap.clawPercentDefault);  // Stop-gap solution -- always passive intake claw whenever Charlie hits this button
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
