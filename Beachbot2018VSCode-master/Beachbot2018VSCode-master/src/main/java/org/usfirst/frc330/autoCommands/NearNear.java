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
public class NearNear extends BBCommandGroup {
	
	double flingDistance = 60+12;
	
	Waypoint wp1 = new Waypoint(0, -182+5, 0);
	Waypoint wp2 = new Waypoint(29, -271+5, 0); //Dropoff at scale
	Waypoint wp3 = new Waypoint(36, -221+5+3, 0); //Drive to second cube
	Waypoint wp4 = new Waypoint(25, -263+5+2, 0); //Drive back to scale
	Waypoint wp5 = new Waypoint(32, -276+5+2, 0); // Second drop off
	Waypoint wp6 = new Waypoint(56+5, -216+5-1, 0); // Pickup third cube
	
	Waypoint wp7 = new Waypoint (40, -208, 0); //Switch Dropoff Location
	

    public NearNear(StartingPosition pos) {
    	
    	boolean invertX = (pos == StartingPosition.LEFT);
    	
    }
}
