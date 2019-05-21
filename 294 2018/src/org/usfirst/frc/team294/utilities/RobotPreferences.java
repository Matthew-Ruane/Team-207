package org.usfirst.frc.team294.utilities;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotPreferences {

	private final Preferences prefs;
	
	/*
	 * all of the robot preferences
	 */
	public boolean inBCRLab;			// Set true if in the BCR lab (with a big pole in the middle of the field)
	public boolean prototypeRobot;		// Set true if using code for prototype bots, false for practice and competition bots
	public boolean driveDirection;		// true for reversed
	public double wheelCircumference;	// wheel circumference, in inches
	public double driveTrainDistanceFudgeFactor;  // Scaling factor for driving distance (default = 1)
	
	public boolean armCalibrated = false;  // Default to arm being uncalibrated.  Calibrate from robot preferences, 
											// "Calibrate arm zero position" button on dashboard,
											// or autocal on low limit switch (see periodic() in armMotor subsystem)
	public double armCalZero;   		// Arm encoder position at O degrees, in encoder ticks (i.e. the calibration factor)
	
	/**
	 * Creates a RobotPreferences object and reads the robot preferences.
	 */
	public RobotPreferences() {
		prefs = Preferences.getInstance();
		refresh();
	}
	
	/**
	 * Re-reads the robot preferences.
	 */
	public void refresh() {
		inBCRLab = prefs.getBoolean("inBCRLab", false);
		prototypeRobot = prefs.getBoolean("prototypeRobot", false); // true if testing code on a prototype, default to false (competition bot)
		driveDirection = prefs.getBoolean("driveDirection", true);
		wheelCircumference = prefs.getDouble("wheelDiameter", 6) * Math.PI;		
		driveTrainDistanceFudgeFactor = prefs.getDouble("driveTrainDistanceFudgeFactor", -9999);
		if (driveTrainDistanceFudgeFactor == -9999) {
			// If fudge factor for driving can't be read, then assume value of 1
			driveTrainDistanceFudgeFactor = 1;  //0.96824;
		}
		
		armCalZero = prefs.getDouble("calibrationZeroDegrees", -9999);
		armCalibrated = (armCalZero != -9999);
		SmartDashboard.putBoolean("Arm Calibrated", armCalibrated);
		if (!armCalibrated) { 
			// If calibration factor for arm can't be read, then don't enable angle control of arm 
			DriverStation.reportError("Error:  Preferences missing from RoboRio for Arm calibration.", true);
			armCalZero = 0;
		} 
	}
	
	/**
	 * Sets arm angle calibration factor and enables angle control modes for arm.
	 * 
	 * @param armCalZero
	 *            Calibration factor for arm
	 * @param writeCalToPreferences
	 *            true = store calibration in Robot Preferences, false = don't
	 *            change Robot Preferences
	 */
	public void setArmCalibration(double armCalZero, boolean writeCalToPreferences) {
		this.armCalZero = armCalZero;
		armCalibrated = true;
		
		// Reset arm target to current value, since we don't want the arm to jump based on the old set angle
		Robot.armMotor.resetPID();
//		Robot.log.writeLogEcho("Arm set calibration,arm cal zero," + armCalZero + ",raw enc," + Robot.armMotor.getArmEncRaw() + ",arm angle," + Robot.armMotor.getArmDegrees());
		
		SmartDashboard.putBoolean("Arm Calibrated", armCalibrated);
		if (writeCalToPreferences) {
			prefs.putDouble("calibrationZeroDegrees", armCalZero);
		}
	}

	public String getString(String k) {
		return getString(k, null);
	}
	public String getString(String k, String d) {
		return prefs.getString(k, d);
	}
	public int getInt(String k) {
		return getInt(k, 0);
	}
	public int getInt(String k, int d) {
		return prefs.getInt(k, d);
	}
	public double getDouble(String k, double d) {
		return prefs.getDouble(k, d);
	}
	public double getDouble(String k) {
		return getDouble(k, 0);
	}
}
