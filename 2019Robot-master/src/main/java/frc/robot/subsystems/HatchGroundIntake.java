/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.ninjaLib.*;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;


/**
 * Add your docs here.
 */


public class HatchGroundIntake extends StatefulSubsystem<HatchGroundIntake.State> {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  VictorSPX feedGroundHatch = new VictorSPX(RobotMap.HATCH_GROUND_INTAKE_PORT);

  public void configMotorController(int timeout){
    feedGroundHatch.config_kP(0, 0, timeout);
    feedGroundHatch.config_kI(0, 0, timeout);
    feedGroundHatch.config_kD(0, 0, timeout);
    feedGroundHatch.config_kF(0, 0, timeout);

    feedGroundHatch.configMotionAcceleration(1000, timeout);
    feedGroundHatch.configMotionCruiseVelocity(1000, timeout);
    feedGroundHatch.setNeutralMode(NeutralMode.Brake);

    feedGroundHatch.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
    //feedArm.config_IntegralZone(0, Constants.intakeArmIZone, timeout);

  }

  public enum State{
    FEED,
    UNFEED,
    STOP
  }


  public int getCurrentPosition() {
    return feedGroundHatch.getSelectedSensorPosition(0);
  }

  public void setPosition(int position){
    feedGroundHatch.set(ControlMode.MotionMagic, position);
  }

  public void setSpeed(double speed){
    feedGroundHatch.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void setState(State state) {
		switch (state) {
      case FEED:
        //runs at 50%
        feedGroundHatch.set(ControlMode.PercentOutput, -1);
        break;
        case UNFEED:
        //runs at 50%
        feedGroundHatch.set(ControlMode.PercentOutput, 0.5);
        break;  
      default:
      case STOP:
      feedGroundHatch.set(ControlMode.PercentOutput,0);
        break;
    }
    }

  @Override
  public void initDefaultCommand() {
    
  }
}

