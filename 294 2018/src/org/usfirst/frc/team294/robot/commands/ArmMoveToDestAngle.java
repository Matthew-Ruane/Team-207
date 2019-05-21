package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToDestAngle extends Command {

	private double destAngle;
	public ToleranceChecker tolcheck;
	
	/**
	 * Move arm to destAngle
	 * @param destAngle in degrees
	 */
	public ArmMoveToDestAngle(double destAngle) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.armMotor);
		this.destAngle = destAngle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		tolcheck = new ToleranceChecker(4, 10);
		destAngle = (destAngle > RobotMap.maxAngle) ? RobotMap.maxAngle : destAngle;
		destAngle = (destAngle < RobotMap.minAngle) ? RobotMap.minAngle : destAngle;
		Robot.log.writeLogEcho("armMoveToDestAngle,destAngle," + destAngle + ",currentAngle," + Robot.armMotor.getArmDegrees());   // armMotor.startPID() logs this, but log here to echo to screen
		Robot.armMotor.startPID(destAngle);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
//		Robot.log.writeLogEcho("armMoveToDestAngle,destAngle," + destAngle + ",currentAngle," + Robot.armMotor.getArmDegrees());
		return (Math.abs( Robot.armMotor.getArmDegrees() - destAngle) <4);
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
