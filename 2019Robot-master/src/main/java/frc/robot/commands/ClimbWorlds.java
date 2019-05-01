package frc.robot.commands;

import frc.robot.HAL;
import frc.robot.subsystems.FrontClimber.FrontClimberPreset;

import frc.robot.subsystems.BackClimber.BackClimberPreset;

import edu.wpi.first.wpilibj.command.Command;

public class ClimbWorlds extends Command {
	private FrontClimberPreset FrontPreset;
	private BackClimberPreset BackPreset;
	private int level;

	public ClimbWorlds(FrontClimberPreset FrontPreset, BackClimberPreset BackPreset, int level) {
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
	
	// protected void execute() {
	// 	if(level == 3){
	// 		if(HAL.backClimber.getCurrentPosition() < 1250){
	// 			HAL.backClimber.setSpeed(0.8);
	// 			}
	// 			else{
	// 				HAL.backClimber.setSpeed(0);
					
	// 			}
	// 			if(HAL.frontClimber.getCurrentPosition() < 3420){
	// 				HAL.frontClimber.setSpeed(0.57);
	// 				}
	// 				else{
	// 					HAL.frontClimber.setSpeed(0);
						
	// 				}
	// 	}
	// 	else{
	// 		if(HAL.backClimber.getCurrentPosition() < 2186){
	// 			HAL.backClimber.setSpeed(0.8);
	// 			}
	// 			else{
	// 				HAL.backClimber.setSpeed(0);
					
	// 			}
	// 			if(HAL.frontClimber.getCurrentPosition() < 3420){
	// 				HAL.frontClimber.setSpeed(0.57);
	// 				}
	// 				else{
	// 					HAL.frontClimber.setSpeed(0);
						
	// 				}
	// 	}
	
	// }

////////////////////
// PRACTICE CLIMB //
////////////////////
	protected void execute() {
        int backPos = HAL.backClimber.getCurrentPosition();
        int frontPos = HAL.frontClimber.getCurrentPosition();
        double backPosd = HAL.backClimber.getCurrentPosition_double();
        double frontPosd = HAL.frontClimber.getCurrentPosition_double();
        double d = new Double(backPos);

		if(backPos < 1300){
            HAL.backClimber.setSpeed(0.9);}
		else{
			HAL.backClimber.setSpeed(0);}
            
        
		if(frontPos < 3477){
            if (frontPosd > 2900){
                HAL.frontClimber.setSpeed(.4);
            }
            else{
            //HAL.frontClimber.setPosition((2*backPos) + 931);}
            HAL.frontClimber.setPosition((1.4517*backPosd) + 931.89);}
            }    
	    else{
			HAL.frontClimber.setSpeed(0);}
	}

	
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
