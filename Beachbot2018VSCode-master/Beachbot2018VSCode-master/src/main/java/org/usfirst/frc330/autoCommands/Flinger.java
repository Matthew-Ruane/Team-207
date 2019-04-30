package org.usfirst.frc330.autoCommands;

import java.util.ArrayList;

import org.usfirst.frc330.autoCommands.Chooser_RightLeftStart.StartingPosition;
import org.usfirst.frc330.commands.*;
import org.usfirst.frc330.commands.commandgroups.*;
import org.usfirst.frc330.commands.drivecommands.*;
import org.usfirst.frc330.constants.ArmConst;
import org.usfirst.frc330.constants.ChassisConst;
import org.usfirst.frc330.constants.HandConst;
import org.usfirst.frc330.constants.LiftConst;
import org.usfirst.frc330.wpilibj.PIDGains;

import edu.wpi.first.wpilibj.command.BBCommand;
import edu.wpi.first.wpilibj.command.BBCommandGroup;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Flinger extends BBCommandGroup {
	
	double flingDistance = 60+12+18;
	
	Waypoint wp1 = new Waypoint(0, -182+5, 0);
	Waypoint wp2 = new Waypoint(29, -274+3, 0); //Dropoff at scale
	Waypoint wp3 = new Waypoint(36, -221+5+3, 0); //Drive to second cube
	Waypoint wp4 = new Waypoint(25, -263+5+2, 0); //Drive back to scale
	Waypoint wp5 = new Waypoint(32, -274, 0); // Second drop off
	Waypoint wp6 = new Waypoint(56+5, -216+5-1, 0); // Pickup third cube

    public Flinger(StartingPosition pos) {
    	
    	if (pos == StartingPosition.LEFT) {
    		wp3.setX(wp3.getX() + 3);
    		wp6.setX(wp6.getX() + 3);
    	}
    	boolean invertX = (pos == StartingPosition.LEFT);
    	
    	
    	//Drive away from wall
    	Command parallelCommand = new DriveWaypointBackward(wp1, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	addParallel(parallelCommand);
    	
    	//addParallel(new Defense());
    	
    	//Finish driving away from wall
    	
    	//Start Drive to Scale
    	addSequential(new TurnGyroWaypointBackward(wp2, invertX, ChassisConst.defaultTurnTolerance, 2, ChassisConst.GyroTurnLow));
    	//PIDGains tempDrive  = new PIDGains(0.10, 0, 0.80, 0, 0.8, ChassisConst.defaultMaxOutputStep, "DriveHigh");
    	parallelCommand = new DriveWaypointBackward(wp2, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	addParallel(parallelCommand);
    	
    	//Fling while driving
    	addSequential(new WaitForPosition(wp2, invertX, flingDistance));
    	   	
    	//Get into pickup position
    	
    	//Go to pickup a cube
    	addSequential(new Log("Pickup Second Cube"));
    	addSequential(new TurnGyroWaypoint(wp3, invertX, ChassisConst.defaultTurnTolerance, 1, ChassisConst.GyroTurnLow));
    	addSequential(new WaitCommand(0.2));
    	addSequential(new DriveWaypoint(wp3, invertX, ChassisConst.defaultTolerance, 5, true, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh));
    	
    	//Grab the cube
    	addSequential(new WaitCommand(0.4));
    	
    	//Return to prep location
    	parallelCommand = new DriveWaypointBackward(wp4, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	addParallel(parallelCommand);
    	addSequential(new WaitCommand(0.4));
    	
    	//Drive to scale for second dropoff
    	addSequential(new TurnGyroWaypointBackward(wp5, invertX, ChassisConst.defaultTurnTolerance, 1, ChassisConst.GyroTurnLow));
    	
    	//Start Flinging
    	addParallel(parallelCommand);
    	
    	addSequential(new DriveWaypointBackward(wp5, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh));
    	addSequential(new WaitCommand(0.1));
    	
    	
    	//Get into pickup position
    	
    	//Go to pickup a new cube
    	addSequential(new TurnGyroWaypoint(wp6, invertX, ChassisConst.defaultTurnTolerance, 1, ChassisConst.GyroTurnLow));
    	addSequential(new DriveWaypoint(wp6, invertX, ChassisConst.defaultTolerance, 5, true, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh));

    	//Grab the cube
    	addSequential(new WaitCommand(0.4));
    	addSequential(new Log("Third cube picked up"));
    	
    	//Return to prep location
    	parallelCommand = new DriveWaypointBackward(wp4, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	addParallel(parallelCommand);
    	addSequential(new WaitCommand(0.6));
    	
    	
    	//Drive to scale for third dropoff
    	addSequential(new TurnGyroWaypointBackward(wp5, invertX, ChassisConst.defaultTurnTolerance, 1, ChassisConst.GyroTurnLow));
    	
    	//Fling Cube
    	addParallel(parallelCommand);
    	
    	addSequential(new DriveWaypointBackward(wp5, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh));
    	addSequential(new WaitCommand(0.1));
    	
       
    }
}
