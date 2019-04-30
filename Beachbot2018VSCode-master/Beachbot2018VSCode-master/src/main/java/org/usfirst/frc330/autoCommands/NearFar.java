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
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class NearFar extends BBCommandGroup {

	
	Waypoint wp1 = new Waypoint(0,   -230+5, 0); //Turn to go down path
	Waypoint wp2 = new Waypoint(212, -230+5, 0); //Drive to scale
	Waypoint wp3 = new Waypoint(195, -279+10+2, 0); //Dropoff at scale
	Waypoint wp4 = new Waypoint(182+2+1, -223+5, 0); //Drive to cube
	Waypoint wp5 = new Waypoint(204, -263+5, 0); //Drive back to scale
	Waypoint wp6 = new Waypoint(192, -276+6+2, 0); //Second scale dropoff
	Waypoint wp7 = new Waypoint(160, -220+5, 0); //Pickup third cube

    public NearFar(StartingPosition pos) {
    	
    	if (pos == StartingPosition.LEFT) {
    		wp4.setX(wp4.getX() + 3);
    		wp7.setX(wp7.getX() + 3);
    	}
    	
    	boolean invertX = (pos == StartingPosition.LEFT);
    	
    	//Close Claw Just in Case
    	Command parallelCommand = new DriveWaypointBackward(wp1, invertX, ChassisConst.defaultTolerance, 5, false, ChassisConst.DriveHigh, ChassisConst.GyroDriveHigh);
    	

    }
}
