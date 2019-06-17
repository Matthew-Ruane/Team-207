// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;


public class DefaultDriveSpark extends CANSparkMax {

	public DefaultDriveSpark(int deviceNumber) {
		super(deviceNumber);

	}

}
