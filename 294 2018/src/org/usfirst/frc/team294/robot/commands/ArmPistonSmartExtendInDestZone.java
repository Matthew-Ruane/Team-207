package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonSmartExtendInDestZone extends Command {

	private double currentAng;
	private ArmZones destZone;
	private boolean done;
	
	/**
	 * Extend piston when the arm enters the zone for destAng
	 * @param destAng destination angle, in degrees
	 */
	public ArmPistonSmartExtendInDestZone(double destAng) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.armPiston);
		destZone = RobotMap.getArmZone(destAng);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		// if (destAng > RobotMap.upperBound || (destAng < RobotMap.middleBound &&
		// destAng > RobotMap.lowerBound)) {
		//done = true;
		// currentAng = Robot.protoArmMotor.getArmDegrees();
		if (destZone == RobotMap.ArmZones.Backwards || destZone == RobotMap.ArmZones.Middle || destZone == RobotMap.ArmZones.Ground)
			done = true;
		else
			done = false;

//		done = false;
	}


	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		currentAng = Robot.armMotor.getArmDegrees();
		
		if (!done && RobotMap.getArmZone(currentAng) == destZone) {
			Robot.armPiston.smartExtend();
			done = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
