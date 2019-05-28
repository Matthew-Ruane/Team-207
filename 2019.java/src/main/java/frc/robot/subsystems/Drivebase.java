package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.utility.*;
import frc.robot.Constants;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import frc.utility.DummyPIDOutput;

/**
 * Author Matt Ruane (207)
 */
public class Drivebase extends Subsystem {

  private static final Drivebase instance = new Drivebase();

  public static Drivebase getInstance() {
    return instance;
  }

  private static DefaultDriveTalonSRX mDrive_Left_Master, mDrive_Left_B, mDrive_Left_C, mDrive_Right_Master, mDrive_Right_B, mDrive_Right_C;

  private static double left, right, lowerbound, upperbound, target;

  public static DifferentialDrive mDrive;

  private static Solenoid mShifter_High, mShifter_Low;

  private static DummyPIDOutput PIDturnOutput;

  public static int leftEncoderZero = 0;
  public static int rightEncoderZero = 0;

  private static double x, y, distance, leftEncoderDistance, prevLeftEncoderDistance, rightEncoderDistance, 
                        prevRightEncoderDistance, gyroAngle, desiredDistanceInches, desiredDistanceTicks,
                        TurnrateCurved;

  private static double setAngle = 0;
  private static double desiredAngle = 0;


  private static double yawZero = 0;
  public static AHRS ahrs;

  public static PIDController PIDturn;


  public Drivebase() {
    mDrive_Left_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
    mDrive_Left_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
    mDrive_Left_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);
    mDrive_Right_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
    mDrive_Right_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
    mDrive_Right_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);

    mDrive_Left_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDrive_Left_Master.setSensorPhase(false);
    mDrive_Left_Master.configMotionCruiseVelocity(Constants.kDriveCruiseVelo);
    mDrive_Left_Master.configMotionAcceleration(Constants.kDriveAccel);

    mDrive_Right_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDrive_Right_Master.setSensorPhase(false);
    mDrive_Right_Master.configMotionCruiseVelocity(Constants.kDriveCruiseVelo);
    mDrive_Right_Master.configMotionAcceleration(Constants.kDriveAccel);

    mDrive_Left_B.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Left_C.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Right_B.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    mDrive_Right_C.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);

    mDrive_Left_Master.config_kP(0, Constants.Drive_kP);
    mDrive_Left_Master.config_kI(0, Constants.Drive_kI);
    mDrive_Left_Master.config_kD(0, Constants.Drive_kD);
    mDrive_Left_Master.config_kF(0, Constants.Drive_kF);

    mDrive_Right_Master.config_kP(0, Constants.Drive_kP);
    mDrive_Right_Master.config_kI(0, Constants.Drive_kI);
    mDrive_Right_Master.config_kD(0, Constants.Drive_kD);
    mDrive_Right_Master.config_kF(0, Constants.Drive_kF);
    
    mDrive = new DifferentialDrive(mDrive_Left_Master, mDrive_Right_Master);
    mDrive.setSafetyEnabled(false);
    
    mShifter_Low = new Solenoid(RobotMap.mPCM_A, RobotMap.mShift_Low_ID);
    mShifter_High = new Solenoid(RobotMap.mPCM_B, RobotMap.mShift_High_ID);
    mShifter_High.set(Constants.On);
    mShifter_Low.set(Constants.Off);
    Constants.CURRENT_GEAR = Constants.HIGH_GEAR;

    ahrs = new AHRS(SerialPort.Port.kMXP);

    PIDturnOutput = new DummyPIDOutput();

    PIDturn = new PIDController(Constants.Turn_kP, Constants.Turn_kI, Constants.Turn_kD, Constants.Turn_kF, ahrs, PIDturnOutput);
    PIDturn.setInputRange(-180.0,  180.0);
    PIDturn.setOutputRange(-0.65, 0.65);
    PIDturn.setAbsoluteTolerance(Constants.kToleranceDegrees);
    PIDturn.setContinuous(true);
  }
  public static void UpShift() {
    mShifter_High.set(Constants.On);
    mShifter_Low.set(Constants.Off);
    Constants.CURRENT_GEAR = Constants.HIGH_GEAR;
  }
  public static void DownShift() {
    mShifter_High.set(Constants.Off);
    mShifter_Low.set(Constants.On);
    Constants.CURRENT_GEAR = Constants.LOW_GEAR;
  }
  public static void tank(double left, double right) {
    mDrive.tankDrive(left, right);
  }
  public static void curvature(double throttleaxis, double turnaxis) {
    TurnrateCurved = (Constants.kTurnrateCurve * Math.pow(turnaxis, 3)+(1-Constants.kTurnrateCurve)*turnaxis*Constants.kTurnrateLimit);
    mDrive.curvatureDrive(throttleaxis, TurnrateCurved, true);
  }
  /*   
  Begin NavX specific Content*/
  public static void zeroYaw() {
    ahrs.zeroYaw();
  }
  public static double getYaw() {
    return ahrs.getYaw();
  }
  public static double getAngle() {
    return ahrs.getAngle();
  }
  /* Enabling volt comp apparently yields better resolution in auto routines but lower max power output. voltage comp config set in talon class */
  public static void EnableVoltComp() {
    mDrive_Left_Master.enableVoltageCompensation(true);
    mDrive_Left_B.enableVoltageCompensation(true);
    mDrive_Left_C.enableVoltageCompensation(true);
    mDrive_Right_Master.enableVoltageCompensation(true);
    mDrive_Right_B.enableVoltageCompensation(true);
    mDrive_Right_C.enableVoltageCompensation(true);
  }
  public static void DisableVoltComp() {
    mDrive_Left_Master.enableVoltageCompensation(false);
    mDrive_Left_B.enableVoltageCompensation(false);
    mDrive_Left_C.enableVoltageCompensation(false);
    mDrive_Right_Master.enableVoltageCompensation(false);
    mDrive_Right_B.enableVoltageCompensation(false);
    mDrive_Right_C.enableVoltageCompensation(false);
  }
  public static void StopDrivetrain() {
    mDrive_Left_Master.set(ControlMode.PercentOutput, 0.0);
    mDrive_Right_Master.set(ControlMode.PercentOutput, 0.0);
  }
  /** @param currentHeading Gyro heading to reset to, in degrees*/
	public static void setGyroRotation(double currentHeading) {
		// set yawZero to gyro angle, offset to currentHeading
    yawZero = -ahrs.getAngle() - currentHeading;
  }
  /*Zeros the gyro position in software*/
	public static void zeroGyroRotation() {
    yawZero = -ahrs.getAngle();
  }
  public static double getGyroRotation() {
    double angle = -ahrs.getAngle() - yawZero;
    /*Angle will be in terms of raw gyro units (-inf,inf), so you need to convert to (-180, 180]*/
    angle = angle % 360;
    angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
    return angle;
  }
  public static void pidTurn() {
    left = -PIDturnOutput.getOutput();
    right = PIDturnOutput.getOutput();
    tank(left, right);
  }
  public static double getTurnOutput() {
    return PIDturnOutput.getOutput();
  }
  public static void motionmagic(double leftTarget, double rightTarget, double turnoutput) {
    mDrive_Left_Master.set(ControlMode.MotionMagic, -leftTarget, DemandType.ArbitraryFeedForward, -turnoutput*0.1);
    mDrive_Right_Master.set(ControlMode.MotionMagic, rightTarget, DemandType.ArbitraryFeedForward, -turnoutput*0.1);
  }
  /** @param current must be pulled directly from actively updating source, generally from getLeftDistance or getRightDistance */
  public static boolean onTargetDistance(double target, double current) {
    lowerbound = Math.abs(target) - Constants.kToleranceDistance;
    upperbound = Math.abs(target) + Constants.kToleranceDistance;
    if (lowerbound <= Math.abs(current) && Math.abs(current) <= upperbound) {
      return true;
    }
    else {
      return false;
    }
  }
  public static void setBrake() {
    mDrive_Left_Master.setNeutralMode(NeutralMode.Brake);
    mDrive_Left_B.setNeutralMode(NeutralMode.Brake);
    mDrive_Left_C.setNeutralMode(NeutralMode.Brake);
    mDrive_Right_Master.setNeutralMode(NeutralMode.Brake);
    mDrive_Right_B.setNeutralMode(NeutralMode.Brake);
    mDrive_Right_C.setNeutralMode(NeutralMode.Brake);
  }
  public static void setCoast() {
    mDrive_Left_Master.setNeutralMode(NeutralMode.Coast);
    mDrive_Left_B.setNeutralMode(NeutralMode.Coast);
    mDrive_Left_C.setNeutralMode(NeutralMode.Coast);
    mDrive_Right_Master.setNeutralMode(NeutralMode.Coast);
    mDrive_Right_B.setNeutralMode(NeutralMode.Coast);
    mDrive_Right_C.setNeutralMode(NeutralMode.Coast);
  }
  public static double DistanceInchesToTicks (double desiredDistanceInches) {
    desiredDistanceTicks = desiredDistanceInches*Constants.encoderTicksPerInch;
    return desiredDistanceTicks;
  }
  public static void resetEncoders() {
    mDrive_Left_Master.setSelectedSensorPosition(0, 0, Constants.kTimeoutms);
    mDrive_Right_Master.setSelectedSensorPosition(0, 0, Constants.kTimeoutms);
  }
  public static void ReportData() {
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("Left encoder", getleftEncoder());
    SmartDashboard.putNumber("right encoder", getrightEncoder());
    SmartDashboard.putNumber("Left encoder rate", getLeftVelocity());
    SmartDashboard.putNumber("right encoder rate", getRightVelocity());
    SmartDashboard.putNumber("turnoutput", getTurnOutput());
  }
  public static int getCurrentGear() {
    return Constants.CURRENT_GEAR;
  }
  public static int getleftEncoder() {
    return mDrive_Left_Master.getSelectedSensorPosition(0);
  }
  public static int getrightEncoder() {
    return mDrive_Right_Master.getSelectedSensorPosition(0);
  }
  public static double getLeftVelocity() {
		return mDrive_Left_Master.getSelectedSensorVelocity(0);
	}
	public static double getRightVelocity() {
		return mDrive_Right_Master.getSelectedSensorVelocity(0);
  }
  public void calcXY() {
		 leftEncoderDistance  = getLeftDistance();
		 rightEncoderDistance = getRightDistance();
		 gyroAngle = getAngle();
		 distance = ((leftEncoderDistance - prevLeftEncoderDistance) + (rightEncoderDistance - prevRightEncoderDistance))/2;
		 x = x + distance * Math.sin(Math.toRadians(gyroAngle));
		 y = y + distance * Math.cos(Math.toRadians(gyroAngle));
		 prevLeftEncoderDistance  = leftEncoderDistance;
		 prevRightEncoderDistance = rightEncoderDistance;
  }
  public double getX() {
		return x;
	}
	public double getY() {
		return y;
  }
	public static void zeroLeftEncoder() {
    leftEncoderZero = mDrive_Left_Master.getSelectedSensorPosition(0);
	}
	public static void zeroRightEncoder() {
		rightEncoderZero = mDrive_Right_Master.getSelectedSensorPosition(0);
	}
	public static int getLeftEncoderTicks() {
    zeroLeftEncoder();
		return mDrive_Left_Master.getSelectedSensorPosition(0) - leftEncoderZero;
	}
	public static int getRightEncoderTicks() {
    zeroRightEncoder();
		return mDrive_Right_Master.getSelectedSensorPosition(0) - rightEncoderZero;
  }
  /* Returns distance in inches */
	public static double getLeftDistance() {
		return getLeftEncoderTicks()*Constants.encoderTicksPerInch;
	}
	public static double getRightDistance() {
		return getRightEncoderTicks()*Constants.encoderTicksPerInch;
  }
  public static void resetPosition() {
    resetEncoders();
    ahrs.reset();
  }
  @Override
  public void initDefaultCommand() {
  }
}
