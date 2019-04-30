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
public class FarNear extends BBCommandGroup {
	
	Waypoint wp1 = new Waypoint(0, -182, 0);
	Waypoint wp2 = new Waypoint(29, -256-15, 0); //Dropoff at scale
	Waypoint wp3 = new Waypoint(38-2, -212-9, 0); //Drive to second cube
	Waypoint wp4 = new Waypoint(29-4, -256-7, 0); //Drive back to scale
	Waypoint wp5 = new Waypoint(29+1, -256-20, 0); // Second drop off
	Waypoint wp6 = new Waypoint(68-15, -218, 0); // Pickup third cube

    public FarNear(StartingPosition pos) {
    	
    	boolean invertX = (pos == StartingPosition.LEFT);
    	
    	//Drive away from wall
    	Command parallelCommand = new DriveWaypointBackward(wp1, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	addParallel(parallelCommand);
    	addSequential(new WaitCommand(0.6));
    	
    	//Lift arm (low CG)
    	addSequential(new WaitCommand(0.1));
    	
    	//Finish driving away from wall
    	
    	//Lift arm to scale dropoff location
    	addParallel(parallelCommand);
    	
    	//Drive to Scale
    	addSequential(new TurnGyroWaypointBackward(wp2, invertX, ChassisConst.defaultTurnTolerance, 2, ChassisConst.GyroTurnLow));
    	DrivePIDGains tempDrive  = new DrivePIDGains(0.10, 0, 0.80, 0, 0.6, ChassisConst.defaultMaxOutputStep, ChassisConst.defaultMinStartOutput, "DriveHigh80%");
    	addSequential(new DriveWaypointBackward(wp2, invertX, ChassisConst.defaultTolerance, 5, false, tempDrive, ChassisConst.GyroDriveHigh));
    	
    	//Aim low and shoot cube
    	addSequential(new WaitCommand(0.1));
    	addSequential(new WaitCommand(0.5));
    	//DO something here!
    	
    	//Get into pickup position
    	//addSequential(new RollerOn());
    	
    	//Go to pickup a cube
    	//addSequential(new Log("Before cube pickup"));
    	addSequential(new TurnGyroWaypoint(wp3, invertX, ChassisConst.defaultTurnTolerance, 2, ChassisConst.GyroTurnLow));
    	addSequential(new WaitCommand(0.2));
    	addSequential(new DriveWaypoint(wp3, invertX, ChassisConst.defaultTolerance, 5, true, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh));
    	
    	//Grab the cube
    	addSequential(new WaitCommand(0.4));
    	
    	//addSequential(new CloseClaw());
    	//addSequential(new WaitCommand(0.6));
    	//Optimization: fire close claw in parallel with driving
    	//addSequential(new Log("Cube picked up"));
    	
    	//Get the cube off the ground
    	
    	//Return to prep location
    	addSequential(new TurnGyroWaypointBackward(wp4, invertX, ChassisConst.defaultTurnTolerance, 2, ChassisConst.GyroTurnLow));
    	//addSequential(new RollerOff()); //Optimization, use a parallel wait command here
    	parallelCommand = new DriveWaypointBackward(wp4, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	addParallel(parallelCommand);
    	addSequential(new WaitCommand(0.4));
    	
    	//Get into dropoff position
    	addParallel(parallelCommand);
    	
    	//Drive to scale for second dropoff
    	addSequential(new TurnGyroWaypointBackward(wp5, invertX, ChassisConst.defaultTurnTolerance, 2, ChassisConst.GyroTurnLow));
    	//addSequential(new WaitCommand(0.1));
    	tempDrive  = new DrivePIDGains(0.10, 0, 0.80, 0, 0.8,ChassisConst.defaultMaxOutputStep, ChassisConst.defaultMinStartOutput, "DriveHigh80%"); //AP 3-12-18
    	addSequential(new DriveWaypointBackward(wp5, invertX, ChassisConst.defaultTolerance, 5, false, tempDrive, ChassisConst.GyroDriveHigh));
    	addSequential(new WaitCommand(0.1));
    	
    	//Aim low and shoot second cube
    	addSequential(new WaitCommand(0.1));
    	addSequential(new WaitCommand(0.4));
    	
    	
    	//Get into pickup position
    	
    	//Go to pickup a new cube
    	addSequential(new TurnGyroWaypoint(wp6, invertX, ChassisConst.defaultTurnTolerance, 2, ChassisConst.GyroTurnLow));
    	addSequential(new DriveWaypoint(wp6, invertX, ChassisConst.defaultTolerance, 5, true, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh));

    	//Grab the cube
    	addSequential(new WaitCommand(0.4));
    	//Optimization: fire close claw in parallel with driving
    	//addSequential(new Log("Cube picked up"));
    	
    	//Get the cube off the ground
    	
    	//Return to prep location
    	parallelCommand = new DriveWaypointBackward(wp4, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	addParallel(parallelCommand);
    	addSequential(new WaitCommand(0.6));
    	//addSequential(new RollerOff());
    	//Get into dropoff position
    	addParallel(parallelCommand);
    	
    	//Drive to scale for third dropoff
    	addSequential(new TurnGyroWaypointBackward(wp5, invertX, ChassisConst.defaultTurnTolerance, 2, ChassisConst.GyroTurnLow));
    	//addSequential(new WaitCommand(0.1));
    	tempDrive  = new DrivePIDGains(0.050,0,0.70,0,0.8,ChassisConst.defaultMaxOutputStep, ChassisConst.defaultMinStartOutput,"DriveHigh80%"); //AP 3-12-18
    	addSequential(new DriveWaypointBackward(wp5, invertX, ChassisConst.defaultTolerance, 5, false, tempDrive, ChassisConst.GyroDriveHigh));
    	addSequential(new WaitCommand(0.1));
    	
    	//Aim low and shoot third cube
    	addSequential(new WaitCommand(0.1));
       
    }
}
