/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotMap;

/**
 * PID Controlled Elevator w/ Talon SRX (2019)
 */
public class Elevator extends Subsystem {
    private static Elevator mInstance = null;
    private double kTargetPositionInches;
    private static double kEncoderTicksPerInch = 1000;
    private double kPosition;

    private TalonSRX mElevator_Master;
    private TalonSRX mElevator_Slave;
    
    public Elevator() {

      //Elevator Drive Motor Config

        mElevator_Master.set(ControlMode.Position, kPosition);
        mElevator_Master.setNeutralMode(NeutralMode.Coast);
        mElevator_Master.setSensorPhase(false);
        mElevator_Master.configContinuousCurrentLimit(39);
        mElevator_Master.configPeakCurrentLimit(0);
        mElevator_Master.enableCurrentLimit(true);
        mElevator_Master.setInverted(false);
        mElevator_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        
      //Elevator PID Config
        
        mElevator_Master.config_kP(1, 10.0);
        mElevator_Master.config_kI(1, 0);
        mElevator_Master.config_kD(1, 20.0);

      //Elevator Follower Motor Config
        mElevator_Slave.set(ControlMode.Follower, 1);
    
    }
    

    public void kPosition () {
        kPosition = kEncoderTicksPerInch*kTargetPositionInches;
    }

    public static Elevator getInstance() {
       return mInstance;
    }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
