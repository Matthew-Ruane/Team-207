package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonSmartRetract extends Command {
	boolean pistonFinal;
	boolean retract;
	double destAngle;

	/**
	 * Retracts piston if needed to keep arm length legal when moving between the
	 * current arm position and destAngle.
	 * 
	 * @param destAngle
	 *            Angle the arm is moving to, in degrees
	 * @param pistonFinal
	 *            Desired piston position when command finishes. True = keep current
	 *            position, false = retract.
	 */
	public ArmPistonSmartRetract(double destAngle, boolean pistonFinal) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.armPiston);
		this.pistonFinal = pistonFinal;
		this.destAngle = destAngle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		this.retract = false;
		ArmZones currentZone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		ArmZones destZone = RobotMap.getArmZone(destAngle);
		if (destZone == ArmZones.Middle || destZone == ArmZones.Backwards || destZone == ArmZones.Ground
				|| (currentZone == ArmZones.Low && destZone == ArmZones.High)
				|| (currentZone == ArmZones.High && destZone == ArmZones.Low) || !pistonFinal) {
			// If arm is in the middle or backwards zone set pistons to retracted
			Robot.armPiston.smartRetract();
			retract = true;
		} else {
//			Robot.armPiston.smartExtend();  // TODO:  Check if this is needed.  We probably should not extend here.
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (!retract)
			return true;

		else if ((Robot.armPiston.getMajor() == PistonPositions.Retracted))
			// && (Robot.armPiston.getMinor() == PistonPositions.Retracted))

			return true;
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}