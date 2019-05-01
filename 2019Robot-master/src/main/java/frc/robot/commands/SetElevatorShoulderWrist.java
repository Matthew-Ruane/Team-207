/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import frc.robot.HAL;
import frc.robot.subsystems.Wrist.WristPreset;
import frc.robot.subsystems.Shoulder.ShoulderPreset;
import frc.robot.subsystems.Elevator.ElevatorPreset;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;

public class SetElevatorShoulderWrist extends Command {
	private WristPreset Wpreset;
	private ShoulderPreset Spreset;
	private ElevatorPreset Epreset;

	DoubleSupplier elevator;
	boolean manual;

	public SetElevatorShoulderWrist(WristPreset Wpreset, ShoulderPreset Spreset, ElevatorPreset Epreset) {
		requires(HAL.elevator);
		requires(HAL.shoulder);
		requires(HAL.wrist);
		this.Epreset= Epreset;
		this.Wpreset = Wpreset;
		this.Spreset = Spreset;
		manual = false;
	}
	public SetElevatorShoulderWrist(DoubleSupplier elevator) {
		requires(HAL.elevator);
		requires(HAL.shoulder);
		requires(HAL.wrist);
		this.elevator = elevator;
		manual = true;
	}
	
	// Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		//when coming up from ground ball, flick the wrist up a bit before setting the desired pos.
		if(HAL.shoulder.getCurrentPosition() < 120 && Spreset.shoulderPosition > 130) {
			HAL.wrist.setPosition(WristPreset.GROUND2TRAVEL); //WRIST parallel to SHOULDER
		} else {
			HAL.wrist.setPosition(Wpreset);
		}
		HAL.shoulder.setPosition(Spreset);
		HAL.elevator.setPosition(Epreset);
	}

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
