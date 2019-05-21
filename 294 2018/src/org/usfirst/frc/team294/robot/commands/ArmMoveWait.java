package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveWait extends Command {

	private double currentAng = Robot.armMotor.getArmDegrees();
	private double destAng;
	private boolean currentPiston;
	private boolean extendPistonAtEnd;

	public ArmMoveWait(double destAng, boolean extendPistonAtEnd) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.destAng = destAng;
		this.extendPistonAtEnd = extendPistonAtEnd;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		currentPiston = Robot.armPiston.getMajor() == RobotMap.PistonPositions.Retracted;
		destAng = (destAng > RobotMap.maxAngle) ? RobotMap.maxAngle : destAng;
		destAng = (destAng < RobotMap.minAngle) ? RobotMap.minAngle : destAng;
		Robot.armMotor.startPID(destAng);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return destAng == currentAng && currentPiston == extendPistonAtEnd;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
