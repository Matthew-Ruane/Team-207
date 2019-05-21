
package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmPositions;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath1_part2_Score2ndCube;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;



public class LoadCubeSequence extends CommandGroup {
	
/**
 * Sequence to load cube with the claw.
 * @param armPosition Arm position to intake with
 * @param pistonPosition Arm extension staet to intake with (intake with arm extended, or retracted?)
 * @param armMoveToSafety If true, move arm to "safe -53 degree position to protect arm. This s
 */
    public LoadCubeSequence(ArmPositions armPosition, PistonPositions pistonPosition, boolean armMoveToSafety) {

    	addSequential(new ArmMoveWithPiston(armPosition, pistonPosition));
    	if(pistonPosition == PistonPositions.Extended) {
    	  	addSequential(new ArmPistonSmartExtend());
    	    
    	}
    	addSequential(new ArmIntakeCube());
    	
    	addSequential(new ConditionalCommand(new ArmPistonRetract(true)) {
			protected boolean condition() {
				return (Robot.claw.getBumpSwitch());
			}
		}); 
    	
    	//if the arm is being 
    	if(armMoveToSafety && Robot.armPiston.getOverride()) {
    		// TODO: Put this back in when the arm retract sensor is fixed.
//    	  	addSequential(new ArmMoveWithPiston(-53, false));  		
    	}
   	}
    
    /**
     * Sequence to load cube with the claw. If no parameter is passed, arm will load at intake position, retract and then move to the "safe" angle (-53 degrees)
     */
    public LoadCubeSequence() {
    	this(ArmPositions.Intake, PistonPositions.Extended, true);
    }
}
