// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Sends only new commands to the Talon to reduce CAN usage.
 */
public class DefaultDriveTalonSRX extends WPI_TalonSRX {
	
	private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	public DefaultDriveTalonSRX(int deviceNumber) {
		super(deviceNumber);
		configFactoryDefault();
		configVoltageCompSaturation(12, 10);
		enableCurrentLimit(true);
		configContinuousCurrentLimit(40);
		configPeakCurrentDuration(100);
		configPeakCurrentLimit(40);
		setNeutralMode(NeutralMode.Coast);
		setSafetyEnabled(false);
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		//return;
		
		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
		} 
	}

	public double getSetpoint() {
		return prevValue;
	}
}
