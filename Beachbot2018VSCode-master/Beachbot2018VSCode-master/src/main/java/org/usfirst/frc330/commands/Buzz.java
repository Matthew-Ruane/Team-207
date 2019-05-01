// RobotBuilder Version: 2.0BB
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc330.commands;
import edu.wpi.first.wpilibj.command.BBCommand;
import org.usfirst.frc330.Robot;
import org.usfirst.frc330.util.Logger;

/**
 *
 */
public class Buzz extends BBCommand {

	double time;
	
    public Buzz(double time) {
    	this.setRunWhenDisabled(true);
    	this.time = time;
    }
    
    public Buzz() {
    	this(1.0);
    }

    protected void initialize() {
    	Robot.buzzer.enable(time);
    	Logger.getInstance().println("Buzzer enabled for: " + time + "seconds", Logger.Severity.INFO);
    }


    protected void execute() {
    }

    protected boolean isFinished() {
        return true;
    }

    protected void end() {
    }


    protected void interrupted() {
    }
}
