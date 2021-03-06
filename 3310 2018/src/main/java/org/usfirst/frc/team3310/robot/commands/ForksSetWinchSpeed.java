package org.usfirst.frc.team3310.robot.commands;

import org.usfirst.frc.team3310.robot.Robot;
import org.usfirst.frc.team3310.robot.subsystems.Elevator.ElevatorSpeedShiftState;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ForksSetWinchSpeed extends Command {
	
	private double speed;

    public ForksSetWinchSpeed(double speed) {
    	this.speed = speed;
        requires(Robot.forks);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.setShiftState(ElevatorSpeedShiftState.LO);
    	Robot.forks.setWinchSpeed(speed);
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
    	Robot.forks.setWinchSpeed(0);
    }
}
