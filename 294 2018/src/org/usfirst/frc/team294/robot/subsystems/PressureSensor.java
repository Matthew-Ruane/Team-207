package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PressureSensor extends Subsystem {
	private AnalogInput pressure = new AnalogInput(RobotMap.pressureSensor);
	private Timer logTimer = new Timer();

	public PressureSensor() {
		logTimer.reset();
		logTimer.start();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public double getVoltage() {
    	double voltage;
    	voltage = pressure.getVoltage();
    	return voltage;
    }
    
    public  int getPressure() {			// integer is all the resolution we need for display
    	double voltage = getVoltage();
    	double inputVoltage = 4.8;
    	double pressure = (250 * (voltage / inputVoltage)) - 25;
    	return (int) pressure;
    }

    public void periodic() {
    	SmartDashboard.putNumber("High Pressure", getPressure()); 
    	
    	if (logTimer.get()>=1.0 && DriverStation.getInstance().isEnabled()) {
    		logTimer.reset();
    		Robot.log.writeLog("Pressure," + getPressure());
    	}
    }
}