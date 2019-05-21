package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeLetGo extends Command {
	
	public CubeLetGo() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.openClaw();
		Robot.claw.setClawMotorPercent(RobotMap.clawPercentLetGo);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
//		Robot.claw.openClaw();
//		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (timeSinceInitialized() >=  0.3) {
			end();
			return true;
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
    	Robot.claw.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
    	Robot.claw.stop();
	}
}
