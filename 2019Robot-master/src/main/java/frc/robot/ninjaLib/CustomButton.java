/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ninjaLib;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.buttons.Button;

public class CustomButton extends Button {

	private final BooleanSupplier custom;

	public CustomButton(BooleanSupplier custom) {
		this.custom = custom;
	}

	@Override
	public boolean get() {
		return this.custom.getAsBoolean();
	}
}
