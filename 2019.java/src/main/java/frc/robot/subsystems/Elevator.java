/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * PID Controlled Elevator w/ Talon SRX (2019)
 * Author Matt Ruane (207)
 */
public class Elevator extends Subsystem {

    private static final Elevator instance = new Elevator();

    public static Elevator getInstance() {
      return instance;
    }

    private static double kEncoderTicksPerInch = 574;
    private static double kPosition = 0;

    //all heights are in units of in
    public static final double COLLECT_CARGO = 2.6;
    public static final double ROCKET_BOTTOM_HEIGHT_CARGO = 5.23;
    public static final double ROCKET_MID_HEIGHT_CARGO = 34.84;
    public static final double ROCKET_TOP_HEIGHT_CARGO = 60.9;
    public static final double SHIP_HEIGHT_CARGO = 19.5;
    public static final double SHIP_HEIGHT_HATCH = 8.3;
    public static final double COLLECT_HATCH = 8.3;
    public static final double ROCKET_BOTTOM_HEIGHT_HATCH = 8.3;
    public static final double ROCKET_MID_HEIGHT_HATCH = 36.5;
    public static final double ROCKET_TOP_HEIGHT_HATCH = 61.8;

    public static enum ElevatorPositions {
      ROCKET_BOTTOM, ROCKET_MID, ROCKET_TOP, CARGO_SHIP, COLLECT;
    }
    public static enum ElevatorModes {
        CARGO, HATCH;
    }
    public static ElevatorModes Mode = ElevatorModes.CARGO;
    public static ElevatorPositions DesiredPosition = ElevatorPositions.ROCKET_BOTTOM;

    private static final TalonSRX mElevator_Master = new TalonSRX(RobotMap.mElevator_Master_ID);
    private static final TalonSRX mElevator_Slave = new TalonSRX(RobotMap.mElevator_Slave_ID);

    public static TalonSRXPIDSetConfiguration pid = new TalonSRXPIDSetConfiguration();
    
    private void Elevator() {

      //Elevator Drive Motor Config

        mElevator_Master.setNeutralMode(NeutralMode.Coast);
        mElevator_Master.setSensorPhase(false);
        mElevator_Master.enableCurrentLimit(true);
        mElevator_Master.configContinuousCurrentLimit(40);
        mElevator_Master.configPeakCurrentLimit(40);
        mElevator_Master.configPeakCurrentDuration(100);
        mElevator_Master.setInverted(false);
        mElevator_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.ElevatorSensorIdx, Constants.kTimeoutMs);
        mElevator_Master.config_kP(Constants.ElevatorSensorIdx, Constants.kElevatorP, Constants.kTimeoutMs);
        mElevator_Master.config_kI(Constants.ElevatorSensorIdx, Constants.kElevatorI, Constants.kTimeoutMs);
        mElevator_Master.config_kD(Constants.ElevatorSensorIdx, Constants.kElevatorD, Constants.kTimeoutMs);
        mElevator_Master.setSelectedSensorPosition(0, Constants.ElevatorSensorIdx, Constants.kTimeoutMs);
        mElevator_Master.config_IntegralZone(Constants.ElevatorSensorIdx, 1000);
        mElevator_Master.configClosedLoopPeakOutput(Constants.ElevatorSensorIdx, 1.0, Constants.kTimeoutMs);
        //Elevator Follower Motor Config
        mElevator_Slave.setNeutralMode(NeutralMode.Coast);
        mElevator_Master.enableCurrentLimit(true);
        mElevator_Slave.configContinuousCurrentLimit(40);
        mElevator_Slave.configPeakCurrentLimit(40);
        mElevator_Slave.configPeakCurrentDuration(100);
        mElevator_Slave.setInverted(false);
        mElevator_Master.set(ControlMode.PercentOutput, 0.0);

    }

    public static void SetElevatorPosition() {
        kPosition = kEncoderTicksPerInch*Elevator.getTargetHeight();
        mElevator_Slave.set(ControlMode.Follower, RobotMap.mElevator_Master_ID);
        mElevator_Master.set(ControlMode.MotionMagic, kPosition);
    }
    public static void SetCargoMode() {
        Mode = ElevatorModes.CARGO;
    }
    public static void SetHatchMode() {
        Mode = ElevatorModes.HATCH;
    }
    public static void zeroElevatorEncoder() {
        mElevator_Master.setSelectedSensorPosition(0);
    }
    public static void stopElevator() {
      mElevator_Master.set(ControlMode.PercentOutput, 0.0);
    }
    public static double getEncoderValue() {
        return mElevator_Master.getSelectedSensorPosition(0);
    }
    //May not work, alternative approach found at following URL: https://github.com/Team254/FRC-2014/blob/master/src/com/team254/frc2014/subsystems/Pinniped.java
    //Above approach implemented for tray
    public static double getTargetHeight() {
        switch (Mode) {
            case CARGO:
              if (DesiredPosition == ElevatorPositions.ROCKET_BOTTOM) {
                return ROCKET_BOTTOM_HEIGHT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_MID) {
                return ROCKET_MID_HEIGHT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_TOP) {
                return ROCKET_TOP_HEIGHT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.COLLECT) {
                return COLLECT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.CARGO_SHIP) {
                return SHIP_HEIGHT_CARGO;
              }
            case HATCH:
              if (DesiredPosition == ElevatorPositions.ROCKET_BOTTOM) {
                return ROCKET_BOTTOM_HEIGHT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_MID) {
                return ROCKET_MID_HEIGHT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_TOP) {
                return ROCKET_TOP_HEIGHT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.COLLECT) {
                return COLLECT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.CARGO_SHIP) {
                return SHIP_HEIGHT_HATCH;
              }
            default:
              return 0;
        }
    }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //  setDefaultCommand(new MySpecialCommand());
  }
}
