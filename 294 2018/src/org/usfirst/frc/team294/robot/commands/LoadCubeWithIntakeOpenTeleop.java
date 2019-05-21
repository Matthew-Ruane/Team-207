
package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;


/**
 * Sequence to load cube from intake to arm and reverse intake motors
 */
public class LoadCubeWithIntakeOpenTeleop extends CommandGroup {

    public LoadCubeWithIntakeOpenTeleop() {

    	/* These commands are all individual because we want them to finish before we continue on to moving anything else, to avoid impacts */
    	

 //   	addSequential(new IntakeSetDeploy(true)); // Deploy the intake first, before anything else
//    	addSequential(new IntakeSetOpen(true)); // Close the claw while moving the arm
    	addSequential(new ConditionalCommand(new ClawSetOpen(false)){
    		protected boolean condition(){
    			return (Robot.armMotor.getArmDegrees()>-45); 
    		}
    		// Move the arm to the intake position
    	});
    	addSequential(new WaitCommand(0.2));
    	addSequential(new LogMessage("loadCube, 1 intake claw set",true)); 
    	addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos,false)); // Move the arm to the intake position
    	addSequential(new LogMessage("loadCube, 2 arm finished moving",true)); 

    	//    		TODO add checker to see if the intake is deployed and open before intaking can occur
    	addSequential(new WaitCommand(0.3));		// Was 0.75 sec
 //   	addParallel(new IntakeCube(false)); // Open intake claw and start intaking, close when the photoswitch is triggered
    	addSequential(new ArmIntakeCube()); // Simultaneously, open the arm claw and begin intaking. Exit when bumpswitch triggered, then lower claw speed to hold cube
	}
}
