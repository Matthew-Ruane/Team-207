package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeShootOut extends Command {

	private double timeShot = 0;
	private double clawPercent;
	private double intakePercent;

	public CubeShootOut() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
//		requires(Robot.intake);
		this.clawPercent = RobotMap.clawPercentShootOut;
		this.intakePercent = RobotMap.intakePercentShootOut;
	}

	/**
	 * Shoot cube at given power
	 * @param percentPower -1 to +1, + to shoot out
	 */
	public CubeShootOut(double percentPower) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
//		requires(Robot.intake);
		this.clawPercent = percentPower;
		this.intakePercent = -percentPower;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.setClawMotorPercent(clawPercent);
//		Robot.intake.setIntakeMotorPercent(intakePercent);
		timeShot = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.claw.setClawMotorPercent(clawPercent);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if ((!Robot.claw.getPhotoSwitch() && Timer.getFPGATimestamp() >= timeShot + 1) || Timer.getFPGATimestamp() >= timeShot + 3) {
			return true;
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
    	Robot.claw.stop();
		//sets intake motor back to its most recent non-zero, inward speed
//		Robot.intake.setIntakeMotorPercent(RobotMap.intakePercentOut);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
