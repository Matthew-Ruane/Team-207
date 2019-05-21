package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMotorControlJoystick extends Command {

	public ArmMotorControlJoystick() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.armMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.log.writeLogEcho("Arm,Joystick control,enabled");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.armMotor.joystickControl=true;
		double armVal = Robot.oi.xboxController.getY();
		Robot.armMotor.setArmMotorToPercentPower(armVal);
		Robot.armMotor.updateSmartDashboard();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.armMotor.joystickControl=false;
		Robot.log.writeLogEcho("Arm,Joystick control,end (disabled)");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.armMotor.joystickControl=false;
		Robot.log.writeLogEcho("Arm,Joystick control,interrupted (disabled)");
	}
}
