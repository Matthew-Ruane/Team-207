package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonRetract extends Command {

	boolean waitForRetract;
	/**
	 * Retracts piston without waiting if not fed a parameter
	 */
    public ArmPistonRetract() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.armPiston);
    	waitForRetract = false;
    }
    
    /**
     * Retracts the arm piston
     * @param waitForRetract if true, wait for arm to retract, if false immediately return
     */
    public ArmPistonRetract(boolean waitForRetract) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.waitForRetract = waitForRetract;
    	requires(Robot.armPiston);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.armPiston.smartRetract();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(!waitForRetract) return true;    	
		else if ((Robot.armPiston.getMajor() == PistonPositions.Retracted))
			return true;
		return false;

    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
