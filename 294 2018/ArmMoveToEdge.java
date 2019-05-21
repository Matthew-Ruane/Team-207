package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToEdge extends Command {
	private double destAngle;
	//private double currentAngle;
	private ArmZones currZone;

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
		case Low:
			if (destAngle >= RobotMap.lowerBound) {
				// We are moving from Low zone to another zone, so move to edge of Low zone
				Robot.armMotor.setArmAngle(RobotMap.lowerBound);
			} else {
				// We aren't leaving Low zone, so just go to destAngle
				Robot.armMotor.setArmAngle(destAngle);
			}
			break;
		case High:
			if (destAngle >= RobotMap.upperBound) {
				// We are moving from High zone to Backward zone, so move to high edge of High zone
				Robot.armMotor.setArmAngle(RobotMap.upperBound);
			} else if (destAngle <= RobotMap.middleBound) {
				// We are moving from High zone to a lower zone, so move to low edge of High zone
				Robot.armMotor.setArmAngle(RobotMap.middleBound); 
			} else {
				// We aren't leaving High zone, so just go to destAngle				
				Robot.armMotor.setArmAngle(destAngle);
			}
			break;
		case Backwards:
			// We are in Middle or Backwards zone, so just go to the dest angle				
			if (RobotMap.getArmZone(destAngle).equals(ArmZones.Backwards) || RobotMap.getArmZone(destAngle).equals(ArmZones.High)) {
				Robot.armMotor.setArmAngle(destAngle);
			} else {
				Robot.armMotor.setArmAngle(RobotMap.middleBound);
			}
			break;
		case Middle:
			// We are in Middle Zone 
			if (!(RobotMap.getArmZone(destAngle) == ArmZones.Backwards)) {
				Robot.armMotor.setArmAngle(destAngle);
			} else {
				Robot.armMotor.setArmAngle(RobotMap.upperBound);
			}
			break;
		}	
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
