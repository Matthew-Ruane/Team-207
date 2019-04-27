package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

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
    
    private void Elevator() {

      //Elevator Drive Motor Config

        mElevator_Master.setNeutralMode(NeutralMode.Coast);
        mElevator_Master.setSensorPhase(false);
        mElevator_Master.enableCurrentLimit(true);
        mElevator_Master.configContinuousCurrentLimit(40);
        mElevator_Master.configPeakCurrentLimit(40);
        mElevator_Master.configPeakCurrentDuration(100);
        mElevator_Master.setInverted(false);
        mElevator_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
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
        Constants.kPosition = Constants.kEncoderTicksPerInch*Elevator.getTargetHeight();
        mElevator_Slave.set(ControlMode.Follower, RobotMap.mElevator_Master_ID);
        mElevator_Master.set(ControlMode.MotionMagic, Constants.kPosition);
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
    public static double getTargetHeight() {
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
