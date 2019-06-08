package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

    public enum ElevatorPositions {
      ROCKET_BOTTOM, ROCKET_MID, ROCKET_TOP, CARGO_SHIP, COLLECT;
    }
    public enum ElevatorModes {
        CARGO, HATCH;
    }
    public ElevatorModes Mode = ElevatorModes.CARGO;
    public ElevatorPositions DesiredPosition = ElevatorPositions.ROCKET_BOTTOM;

    public TalonSRX mElevator_Master, mElevator_Slave;
    
    public Elevator() {
      mElevator_Master = new TalonSRX(RobotMap.mElevator_Master_ID);
      mElevator_Slave = new TalonSRX(RobotMap.mElevator_Slave_ID);
      mElevator_Master.configFactoryDefault();
      mElevator_Master.setNeutralMode(NeutralMode.Coast);
      mElevator_Master.setSensorPhase(false);
      mElevator_Master.enableCurrentLimit(true);
      mElevator_Master.configContinuousCurrentLimit(40);
      mElevator_Master.configPeakCurrentLimit(40);
      mElevator_Master.configPeakCurrentDuration(100);
      mElevator_Master.setInverted(false);
      mElevator_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
      mElevator_Master.config_kP(Constants.ElevatorSlotIDx, Constants.Elevator_kP, Constants.kTimeoutms);
      mElevator_Master.config_kI(Constants.ElevatorSlotIDx, Constants.Elevator_kI, Constants.kTimeoutms);
      mElevator_Master.config_kD(Constants.ElevatorSlotIDx, Constants.Elevator_kD, Constants.kTimeoutms);
      mElevator_Master.configMotionAcceleration(Constants.Elevator_MotionAccel, Constants.kTimeoutms);
      mElevator_Master.configMotionCruiseVelocity(Constants.Elevator_MotionCruiseVelo, Constants.kTimeoutms);

      
      mElevator_Slave.configFactoryDefault();
      mElevator_Slave.setNeutralMode(NeutralMode.Coast);
      mElevator_Slave.enableCurrentLimit(true);
      mElevator_Slave.configContinuousCurrentLimit(40);
      mElevator_Slave.configPeakCurrentLimit(40);
      mElevator_Slave.configPeakCurrentDuration(100);
      mElevator_Slave.setInverted(false);
      mElevator_Slave.set(ControlMode.Follower, RobotMap.mElevator_Master_ID);
    }
    public void SetElevatorPosition(ElevatorPositions position, ElevatorModes mode) {
        DesiredPosition = position;
        Mode = mode;
        Constants.kPosition = Constants.kEncoderTicksPerInch*getTargetHeight();
        mElevator_Master.set(ControlMode.MotionMagic, Constants.kPosition);
    }
    public ElevatorPositions GetElevatorPosition() {
      return DesiredPosition;
    }
    public void SetCargoMode() {
        Mode = ElevatorModes.CARGO;
    }
    public void SetHatchMode() {
        Mode = ElevatorModes.HATCH;
    }
    public void SetMode(ElevatorModes mode) {
      Mode = mode;
    }
    public ElevatorModes getMode() {
      return Mode;
    }
    public void zeroElevatorEncoder() {
        mElevator_Master.setSelectedSensorPosition(0);
    }
    public void stopElevator() {
      mElevator_Master.set(ControlMode.PercentOutput, 0.0);
    }
    public double getEncoderValue() {
        return mElevator_Master.getSelectedSensorPosition(0);
    }
    public double getTargetHeight() {
        switch (Mode) {
            case CARGO:
              if (DesiredPosition == ElevatorPositions.ROCKET_BOTTOM) {
                return Constants.ROCKET_BOTTOM_HEIGHT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_MID) {
                return Constants.ROCKET_MID_HEIGHT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_TOP) {
                return Constants.ROCKET_TOP_HEIGHT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.COLLECT) {
                return Constants.COLLECT_CARGO;
              }
              else if (DesiredPosition == ElevatorPositions.CARGO_SHIP) {
                return Constants.SHIP_HEIGHT_CARGO;
              }
            case HATCH:
              if (DesiredPosition == ElevatorPositions.ROCKET_BOTTOM) {
                return Constants.ROCKET_BOTTOM_HEIGHT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_MID) {
                return Constants.ROCKET_MID_HEIGHT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.ROCKET_TOP) {
                return Constants.ROCKET_TOP_HEIGHT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.COLLECT) {
                return Constants.COLLECT_HATCH;
              }
              else if (DesiredPosition == ElevatorPositions.CARGO_SHIP) {
                return Constants.SHIP_HEIGHT_HATCH;
              }
            default:
              return 0;
        }
    }
  @Override
  public void initDefaultCommand() {
  }
}
