package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmIntakeCube extends Command {
	
	private double timeClawClosed = 10000.0;
	private boolean photoSwitchTriggered = false;

    public ArmIntakeCube() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.claw);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	// initialize variables
    	timeClawClosed = 10000.0;
    	photoSwitchTriggered = false;
    	
    	//set the claw to intake
    	Robot.claw.openClaw();
    	Robot.claw.setClawMotorPercent(RobotMap.clawPercentIn);
    	//actuate claw jaws
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putBoolean("Arm Smart Close Return", Robot.claw.getPhotoSwitch());
    	
    	// Close the claw on the arm when the photoswitch is triggered
    	if (Robot.claw.getPhotoSwitch()||Robot.claw.getBumpSwitch()) {
    		Robot.claw.closeClaw();
    		if(!photoSwitchTriggered) {
    			photoSwitchTriggered = true;
    			timeClawClosed = Timer.getFPGATimestamp();
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// if the claw has been closed 
    	Robot.log.writeLog("ArmIntakeCube,bumpswitch," + Robot.claw.getBumpSwitch() + ",photoswitch," + Robot.claw.getPhotoSwitch()
    						+ "switchTriggered," + photoSwitchTriggered 
    						+ ",time Claw Closed," + timeClawClosed + ",timer," + Timer.getFPGATimestamp());
    	if (Robot.claw.getBumpSwitch() || (photoSwitchTriggered && (Timer.getFPGATimestamp() >= (timeClawClosed + 2))) ) {
    		end();
    		return true;
    	} else {
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.claw.setClawMotorPercent(RobotMap.clawPercentDefault);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//stop claw motors
    	Robot.claw.setClawMotorPercent(RobotMap.clawPercentDefault);    }
}
