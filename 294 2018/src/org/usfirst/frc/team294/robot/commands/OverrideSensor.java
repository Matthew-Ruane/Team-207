package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.ref.Reference;
import java.lang.ref.SoftReference;

import org.usfirst.frc.team294.robot.Robot;
/**
 *
 */
public class OverrideSensor extends Command {
	public static enum Sensors{
		Bump,
		ArmPiston,	
		Both,
		None
	}

	private Sensors sensor = Sensors.None;
	private boolean useSmartDashboard;
	
	/**
	 * Overrides the selected sensor when the command is run
	 * @param sensor Sensors.Bump, ArmPiston, Both, or None
	 */
    public OverrideSensor(Sensors sensor) {
    	this.sensor = sensor;
    	useSmartDashboard = false;
    }
    
    /**
     * Overrides sensors based on the selection from SmartDashboard
     */
    public OverrideSensor() {
    	useSmartDashboard = true;
     }

    /**
     * Sets the overrides for the currently selected sensors (per the initializier or SmartDashboard)
     */
    public void setOverrides() {
    	// Read from SmartDashboard, if selected
    	if (useSmartDashboard) {
           	if(SmartDashboard.getBoolean("Override Arm Retract Sensor", false)||SmartDashboard.getBoolean("Override Bump Switch", false)) {
        		sensor = Sensors.Both;
        	}else if(SmartDashboard.getBoolean("Override Arm Retract Sensor", false)){
        		sensor = Sensors.ArmPiston;
        	}else if(SmartDashboard.getBoolean("Override Bump Switch", false)) {
        		sensor = Sensors.Bump;
        	}else {
        		sensor = Sensors.None;
        	}   	    		
    	}

    	// Set selected overrides
    	switch(sensor) {
    	case Both:
    		Robot.claw.overrideBumpSwitch(true);
    		Robot.armPiston.overrideArmSensor(true);
    		break;
    	case Bump:
    		Robot.claw.overrideBumpSwitch(true);
    		Robot.armPiston.overrideArmSensor(false);
    		break;
    	case ArmPiston:
    		Robot.claw.overrideBumpSwitch(false);
    		Robot.armPiston.overrideArmSensor(true);
    		break;
    	case None:
    		Robot.claw.overrideBumpSwitch(false);
    		Robot.armPiston.overrideArmSensor(false);
    		break;
    	}
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
    	setOverrides();
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
