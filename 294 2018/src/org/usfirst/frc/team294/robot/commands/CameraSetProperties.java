package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CameraSetProperties extends Command {

	/**
	 * Sets drive camera properties from SmartDashboard.
	 */
    public CameraSetProperties() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	UsbCamera driveCamera = Robot.driveCamera;

    	int exposure = ((int)SmartDashboard.getNumber("Camera Exposure (-1 auto)" , 30));
    	int brightness = ((int)SmartDashboard.getNumber("Camera Brightness" , 30));
    	
		driveCamera.setExposureAuto(); // Start in auto exposure mode so that we can set brightness 
		driveCamera.setBrightness(brightness); // Setting brightness only works correctly in auto exposure mode (?)  was 10
//		driveCamera.getProperty("contrast").set(80);
//		driveCamera.getProperty("saturation").set(60);
//		driveCamera.setWhiteBalanceManual(2800);
		if (exposure>0) driveCamera.setExposureManual(exposure);
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
