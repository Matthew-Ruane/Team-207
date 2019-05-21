package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;
import org.usfirst.frc.team294.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToEdge extends Command {
	private double destAngle;
	// private double currentAngle;
	private ArmZones currZone;
	public ToleranceChecker tolcheck;

	/**
	 * Moves arm to the edge of the current zone, towards destAngle. Use this to
	 * start the arm moving while waiting for a piston to retract.
	 * 
	 * @param destAngle
	 *            in degrees
	 */
	public ArmMoveToEdge(double destAngle) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.destAngle = destAngle;
		requires(Robot.armMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		currZone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		switch (currZone) {
		case Ground:
			break;
		case Low:
			if (destAngle >= RobotMap.lowerBound) {
				// We are moving from Low zone to another zone, so move to edge of Low zone
				destAngle = RobotMap.lowerBound;
			} else if (destAngle <= RobotMap.groundBound) {
				destAngle = RobotMap.groundBound;
			} else {
				// We aren't leaving Low zone, so just go to destAngle
			}
			break;
		case High:
			if (destAngle >= RobotMap.upperBound) {
				// We are moving from High zone to Backward zone, so move to high edge of High
				// zone
				destAngle = RobotMap.upperBound;
			} else if (destAngle <= RobotMap.middleBound) {
				// We are moving from High zone to a lower zone, so move to low edge of High
				// zone
				destAngle = RobotMap.middleBound;
			} else {
				// We aren't leaving High zone, so just go to destAngle
			}
			break;
		case Backwards:
			// We are in Backwards zone, so just go to the dest angle
			break;
		case Middle:
			// We are in Middle zone, so just go to the dest angle
			break;
		}

		// Move to the new destAngle (edge of zone)
		Robot.armMotor.startPID(destAngle);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (Math.abs(Robot.armMotor.getArmDegrees() - destAngle) < 4);
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
