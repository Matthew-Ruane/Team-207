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
public class NearSwitch extends BBCommandGroup {
	
	double flingDistance = 60+12;
	
	Waypoint wp1 = new Waypoint(0, -15*12, 0); //Drive away from wall to center switch
	Waypoint wp2 = new Waypoint(12, -14.5*12, 0); //Dropoff at switch
	Waypoint wp3 = new Waypoint(25, -218+10, 0); //Pickup second cube
	Waypoint wp7 = new Waypoint(36, -221+18, 0); //Deploy second cube
	Waypoint wp4 = new Waypoint(14, -240, 0); //Drive to prep location 1
	Waypoint wp8 = new Waypoint(55, -230, 0); //Drive to prep location 2
	Waypoint wp5 = new Waypoint(32, -276+5+2, 0); // Second drop off
	Waypoint wp6 = new Waypoint(47, -213, 0); // Pickup third cube

    public NearSwitch(StartingPosition pos) {
    	
    	boolean invertX = (pos == StartingPosition.LEFT);
    	
    }
}
