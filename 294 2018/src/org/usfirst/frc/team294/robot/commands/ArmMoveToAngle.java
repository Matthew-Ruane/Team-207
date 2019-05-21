package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmPositions;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToAngle extends Command {

	double angle;
	
    public ArmMoveToAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.armMotor);
    	if (angle > RobotMap.maxAngle) angle = RobotMap.maxAngle;
    	if (angle < RobotMap.minAngle) angle = RobotMap.minAngle;
    	this.angle = angle;
    }
    
    public ArmMoveToAngle(ArmPositions position) {
    	angle = position.getAngle();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.armMotor.startPID(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true; // This will need some tolerance checking, probably
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
