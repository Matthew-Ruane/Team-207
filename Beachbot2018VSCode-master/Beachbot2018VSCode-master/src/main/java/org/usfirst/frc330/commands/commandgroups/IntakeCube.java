// RobotBuilder Version: 2.0BB
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc330.commands.commandgroups;

import edu.wpi.first.wpilibj.command.BBCommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import org.usfirst.frc330.commands.*;
import org.usfirst.frc330.constants.*;
import org.usfirst.frc330.subsystems.*;
import org.usfirst.frc330.Robot;

/**
 *
 */
public class IntakeCube extends BBCommandGroup {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public IntakeCube() {

    	
    	//if the cube is in the grabber, loop until it is gone
    	addSequential(new LoopIfWeHaveCube());
    	//if the grabber has no cube in it, execute the following code:
    	
    	//Prep Grabber
    	addSequential(new OpenClaw());
    	addSequential(new RollerOn());
    	
    	//lower lift and arm to intake position
    	addParallel(new SetLiftPosition(LiftConst.intakePosition));
    	addSequential(new CoordinatedMove(ArmConst.intakePosition, HandConst.pickUp));
    	
    	//Close claw once we have the cube
    	addSequential(new WaitCommand(0.45)); //Consider shortening
    	addSequential(new SensorCloseClaw());
    	
    	// Once everything above has been completed and the cube is in the grabber...
    	addSequential(new RollerUntilCube(0.6, 2.0));
    	
    	// DEFENSE MODE!  
    	addSequential(new Defense());
    	addSequential(new IsFinishedFalse());
    } 
}