
package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team294.robot.OI;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * The Only Command
 * runs Drive Train with Both Joysticks
 */
public class DriveWithJoysticks extends Command {
	public DriveWithJoysticks() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
//		if(Math.abs(OI.leftJoystick.getY())>.15) {//Dead Zone of 15%
//			Robot.driveTrain.setLeftMotors(OI.leftJoystick.getY()); //sets power of left motors based off of 
//		}else {
//			Robot.driveTrain.setLeftMotors(0);
//		}
//		if(Math.abs(OI.rightJoystick.getY())>.15) {
//			Robot.driveTrain.setRightMotors(OI.rightJoystick.getY());
//		}else {
//			Robot.driveTrain.setRightMotors(0);
//		}
		double leftVal = Robot.oi.leftJoystick.getY();
		double rightVal = Robot.oi.rightJoystick.getY();
		Robot.driveTrain.getGyroRotation();
		Robot.driveTrain.getLeftEncoderTicks();
		Robot.driveTrain.getRightEncoderTicks();
		
    	if (Robot.oi.getDriveDirection() == true)
    		Robot.driveTrain.tankDrive(-leftVal, -rightVal);
    	else
    		Robot.driveTrain.tankDrive(rightVal, leftVal);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.driveTrain.stopAllMotors();
	}
}
