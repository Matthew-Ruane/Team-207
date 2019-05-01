package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.HAL;
import frc.robot.RobotMap;
import frc.robot.commands.SwitchPipeline;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class SnakeLime extends Command {
  boolean kill;
  public SnakeLime() {
    requires(HAL.limelight);
    requires(HAL.drive);

    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(!RobotMap.vision){
      HAL.limelight.setPipeline(6);}
    else{
      HAL.limelight.setPipeline(8);}
      RobotMap.vision = !RobotMap.vision;
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    
    //HAL.limelight.setPipeline(7);
    double xOffset = HAL.limelight.xOffset();
    double turPos = HAL.turret.getCurrentPosition();
    double area = HAL.limelight.area();
    boolean isTarget = HAL.limelight.hasTargets();
    double minArea = 1.3;
    /*if(area <= 0.01){

      
      HAL.limelight.setPipeline(6);
      area = HAL.limelight.area();
    }
    */

    ////////////////////
    // FRONT POSITION //
    ////////////////////
    if (turPos > -6000 && turPos < 6000) {      
      // OFFSET NEGATIVE IS TO THE LEFT
      // OFFSET POSITIVE IS TO THE RIGHT
      if((area > 0.001) && (area <= 1.4)){
        HAL.drive.arcade(-0.5, -(xOffset+1.5)/28); //xOffset+1.0) //COMP is -2
      }
      
      else if(kill && area <= minArea){
        HAL.drive.leftMaster.set(0.0);
        HAL.drive.rightMaster.set(0.0);
        end();
      }

      else if((area > 0.001) && (area <= minArea)){
        kill = true;
        HAL.drive.arcade(-0.5, -(xOffset+1.5)/28); //xOffset+1.0) //COMP is -2
      }

      else if ((area > minArea) && (area <= 5)){
        HAL.turret.turret.set(ControlMode.PercentOutput, ((HAL.limelight.xOffset())/27));
        HAL.drive.leftMaster.set(0.0);
        HAL.drive.rightMaster.set(0.0);
        if ((HAL.shoulder.getCurrentPosition() > 100) && (HAL.shoulder.getCurrentPosition() < 350)){
          int curTurPos = HAL.turret.getCurrentPosition();
          HAL.turret.setPosition(curTurPos);
        }
      }
      
      else {
        HAL.drive.leftMaster.set(0.0);
        HAL.drive.rightMaster.set(0.0);
        end();
      }
    } 

    ///////////////////
    // BACK POSITION //
    ///////////////////
    else if ((turPos > 27000 && turPos < 37000) || (turPos < -27000 && turPos > -37000) ) {

      /*(xOffset > 1 || xOffset < -1) && */
      if((area > 0.01) && (area <= minArea)){
        HAL.drive.arcade(0.5, (-xOffset+2)/27); //offset +1
      }

      else if ((area > minArea) && (area <= 1.6)){
        //HAL.turret.turret.set(ControlMode.PercentOutput, ((HAL.limelight.xOffset())/27));
        HAL.drive.leftMaster.set(0.0);
        HAL.drive.rightMaster.set(0.0);
      }
      
      else {
        HAL.drive.leftMaster.set(0.0);
        HAL.drive.rightMaster.set(0.0);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {


    // if (HAL.limelight.xOffset() > -1 && HAL.limelight.xOffset() > -1 && HAL.limelight.area() > 1.5){
    //   return true;
    // }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //HAL.limelight.setPipeline(8);
    // int turCurPos = HAL.turret.getCurrentPosition();
    // HAL.turret.setPosition(turCurPos);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //HAL.limelight.setPipeline(8);
    end();
  }
}
