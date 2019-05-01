package frc.robot.commands;

import frc.robot.HAL;
import frc.robot.subsystems.FrontClimber.FrontClimberPreset;

import frc.robot.subsystems.BackClimber.BackClimberPreset;

import edu.wpi.first.wpilibj.command.Command;

public class Climb extends Command {
	private FrontClimberPreset FrontPreset;
	private BackClimberPreset BackPreset;
	private int level;

	public Climb(FrontClimberPreset FrontPreset, BackClimberPreset BackPreset, int level) {
		requires(HAL.frontClimber);
		requires(HAL.backClimber);
		this.FrontPreset= FrontPreset;
		this.BackPreset = BackPreset;
		this.level=level;
	}
	
	// Called just before this Command runs the first time
    protected void initialize() {
		
		
  	//System.out.println("Now Wants lift:" +desiredElevatorPos+"Wants Wrist:"+desiredWristPos);
    }

    // Called repeatedly when this Command is scheduled to run
	
	///////////////////////
	// COMPETITION CLIMB //
	///////////////////////
	
	protected void execute() {
		if(level == 3){
			if(HAL.backClimber.getCurrentPosition() < 1250){
				HAL.backClimber.setSpeed(0.8);
				}
				else{
					HAL.backClimber.setSpeed(0);
					
				}
				if(HAL.frontClimber.getCurrentPosition() < 3420){
					HAL.frontClimber.setSpeed(0.61);
					}
					else{
						HAL.frontClimber.setSpeed(0);
						
					}
		}
		else{
			if(HAL.backClimber.getCurrentPosition() < 2186){
				HAL.backClimber.setSpeed(0.8);
				}
				else{
					HAL.backClimber.setSpeed(0);
					
				}
				if(HAL.frontClimber.getCurrentPosition() < 3420){
					HAL.frontClimber.setSpeed(0.47);
					}
					else{
						HAL.frontClimber.setSpeed(0);
						
					}
		}
	
	}

////////////////////
// PRACTICE CLIMB //
////////////////////
	// protected void execute() {
		
	// 	if(HAL.backClimber.getCurrentPosition() < 1300){
	// 		HAL.backClimber.setSpeed(1); 
	// 		}
	// 		else{
	// 			HAL.backClimber.setSpeed(0);
				
	// 		}
	// 		if(HAL.frontClimber.getCurrentPosition() < 3477){
	// 			HAL.frontClimber.setSpeed(0.3);
	// 			}
	// 			else{
	// 				HAL.frontClimber.setSpeed(0);
					
	// 			}
	// }

	
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
