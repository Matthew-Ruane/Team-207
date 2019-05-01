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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class BallFeeder extends StatefulSubsystem<BallFeeder.State> {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  VictorSPX feedBall = new VictorSPX(RobotMap.FEEDER_BALL_PORT);

  public void configMotorController(int timeout){
    feedBall.config_kP(0, 0, timeout);
    feedBall.config_kI(0, 0, timeout);
    feedBall.config_kD(0, 0, timeout);
    feedBall.config_kF(0, 0, timeout);

    feedBall.configMotionAcceleration(4000, timeout);
    feedBall.configMotionCruiseVelocity(4000, timeout);
    feedBall.setNeutralMode(NeutralMode.Brake);

    feedBall.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
    //feedArm.config_IntegralZone(0, Constants.intakeArmIZone, timeout);

  }

  public enum State{
    FEED,
    UNFEED,
    STOP
  }

  
  public BallFeeder()
  {
   
  }

  public int getCurrentPosition() {
    return feedBall.getSelectedSensorPosition(0);
  }

  public void setPosition(int position){
    feedBall.set(ControlMode.MotionMagic, position);
  }

  public void setSpeed(double speed){
    feedBall.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void setState(State state) {
		switch (state) {
      case FEED:
        //runs at 50%
        feedBall.set(ControlMode.PercentOutput, 1.0); //normally 0.8 SoFlo
        break;
        case UNFEED:
        //runs at 50%
        feedBall.set(ControlMode.PercentOutput, -1.0); //normally -0.5
        break;  
      default:
      case STOP:
        feedBall.set(ControlMode.PercentOutput,0);
        break;
    }
    }

  @Override
  public void initDefaultCommand() {
    
  }
}
