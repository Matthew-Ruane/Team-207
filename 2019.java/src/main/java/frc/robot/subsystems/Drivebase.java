package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
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
import frc.utility.PurePursuit.*;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;

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

  public static DefaultDriveTalonSRX mDrive_Left_Master, mDrive_Left_B, mDrive_Left_C, mDrive_Right_Master, mDrive_Right_B, mDrive_Right_C;

  private static double leftpow_, rightpow_;

  public static DifferentialDrive mDrive;

  private static Solenoid mShifter_High, mShifter_Low;

  public static DummyPIDOutput PIDturnOutput, PIDleftOutput, PIDrightOutput;

  protected static final int kVelocityControlSlot = 0;
  protected static final int kBaseLockControlSlot = 1;

  private double TurnrateCurved, mLastHeadingErrorDegrees, leftvelo_,  rightvelo_, left_distance, right_distance, time;
  public double left_encoder_prev_distance_, right_encoder_prev_distance_;

  public static enum DriveControlState {OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL }
  
  private static DriveControlState driveControlState_;
  
  private VelocityHeadingSetpoint velocityHeadingSetpoint_;

  private AHRS ahrs;

  public static PIDController PIDturn;

  private static AdaptivePurePursuitController pathFollowingController_;
  private static SynchronousPID velocityHeadingPid_;

  private Rotation2d gyro_angle;
  private RigidTransform2d odometry;
  private RigidTransform2d.Delta velocity;
  private RobotState robotstate = RobotState.getInstance();


  public Drivebase() {
    mDrive_Left_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
    mDrive_Left_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
    mDrive_Left_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);
    mDrive_Right_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
    mDrive_Right_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
    mDrive_Right_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);

    mDrive_Left_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDrive_Left_Master.setSensorPhase(false);
    mDrive_Left_Master.setInverted(false);
    mDrive_Left_B.setInverted(false);
    mDrive_Left_C.setInverted(false);

    mDrive_Right_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDrive_Right_Master.setSensorPhase(false);
    mDrive_Right_Master.setInverted(false);
    mDrive_Right_B.setInverted(false);
    mDrive_Right_C.setInverted(false);

    mDrive_Left_B.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Left_C.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Right_B.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    mDrive_Right_C.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);

    mDrive_Left_Master.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp);
    mDrive_Left_Master.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi);
    mDrive_Left_Master.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd);
    mDrive_Left_Master.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf);
    mDrive_Left_Master.config_IntegralZone(kVelocityControlSlot, Constants.Drive_kIzone);

    mDrive_Right_Master.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp);
    mDrive_Right_Master.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi);
    mDrive_Right_Master.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd);
    mDrive_Right_Master.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf);
    mDrive_Right_Master.config_IntegralZone(kVelocityControlSlot, Constants.Drive_kIzone);

    velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi, Constants.kDriveHeadingVelocityKd);
    velocityHeadingPid_.setOutputRange(-30, 30);
    
    mDrive = new DifferentialDrive(mDrive_Left_Master, mDrive_Right_Master);
    mDrive.setSafetyEnabled(false);
    
    mShifter_Low = new Solenoid(RobotMap.mPCM_A, RobotMap.mShift_Low_ID);
    mShifter_High = new Solenoid(RobotMap.mPCM_B, RobotMap.mShift_High_ID);

    ahrs = new AHRS(SerialPort.Port.kMXP);

    PIDturnOutput = new DummyPIDOutput();

    PIDturn = new PIDController(Constants.Turn_kP, Constants.Turn_kI, Constants.Turn_kD, Constants.Turn_kF, ahrs, PIDturnOutput, 0.02);
    PIDturn.setInputRange(-180.0,  180.0);
    PIDturn.setOutputRange(-0.65, 0.65);
    PIDturn.setAbsoluteTolerance(Constants.kToleranceDegrees);
    PIDturn.setContinuous(true);
  }
  public void UpShift() {
    mShifter_High.set(Constants.On);
    mShifter_Low.set(Constants.Off);
    Constants.CURRENT_GEAR = Constants.HIGH_GEAR;
  }
  public void DownShift() {
    mShifter_High.set(Constants.Off);
    mShifter_Low.set(Constants.On);
    Constants.CURRENT_GEAR = Constants.LOW_GEAR;
  }
  public void tank(double left, double right) {
    mDrive.tankDrive(left, right);
  }
  public void curvature(double throttleaxis, double turnaxis) {
    TurnrateCurved = (Constants.kTurnrateCurve * Math.pow(turnaxis, 3)+(1-Constants.kTurnrateCurve)*turnaxis*Constants.kTurnrateLimit);
    mDrive.curvatureDrive(throttleaxis, TurnrateCurved, true);
  }
  /* PURE PURSUIT */
  public void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
    configureTalonsForSpeedControl();
    updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
  }
  public void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
  configureTalonsForSpeedControl();
  velocityHeadingPid_.reset();
  velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec, headingSetpoint);
  updateVelocityHeadingSetpoint();
  }
  public void followPath(Path path, boolean reversed) {
  configureTalonsForSpeedControl();
  velocityHeadingPid_.reset();
  pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
    Constants.kPathFollowingMaxAccel, Constants.kLooperDt, path, reversed, 0.25);
  updatePathFollower();
  }
  private void configureTalonsForSpeedControl() {
    mDrive_Left_Master.set(ControlMode.Velocity, 0.0);
    mDrive_Left_Master.selectProfileSlot(kVelocityControlSlot, 0);
    mDrive_Left_Master.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
    mDrive_Right_Master.set(ControlMode.Velocity, 0.0);
    mDrive_Right_Master.selectProfileSlot(kVelocityControlSlot, 0);
    mDrive_Right_Master.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
    setBrake();
  }
  public void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
    leftvelo_ = inchesPerSecondToVelo(left_inches_per_sec);
    rightvelo_ = inchesPerSecondToVelo(right_inches_per_sec);
    mDrive_Left_Master.set(ControlMode.Velocity, -leftvelo_);
    mDrive_Right_Master.set(ControlMode.Velocity, rightvelo_);
  }
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(ahrs.getAngle());
  }
  private void updateVelocityHeadingSetpoint() {
    Rotation2d actualGyroAngle = getGyroAngle();

    mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse()).getDegrees();

    double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
    updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2, velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
  }
public void updatePathFollower() {
  RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
  RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
  Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

  // Scale the command to respect the max velocity limits
  double max_vel = 0.0;
  max_vel = Math.max(max_vel, Math.abs(setpoint.left));
  max_vel = Math.max(max_vel, Math.abs(setpoint.right));
  if (max_vel > Constants.kPathFollowingMaxVel) {
      double scaling = Constants.kPathFollowingMaxVel / max_vel;
      setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
  }
  updateVelocitySetpoint(setpoint.left, setpoint.right);
}
public void updateRobotState() {
  time = Timer.getFPGATimestamp();
  left_distance = getLeftDistanceInches();
  right_distance = getRightDistanceInches();
  gyro_angle = getGyroAngle();
  odometry = robotstate.generateOdometryFromSensors(left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
  velocity = Kinematics.forwardKinematics(getLeftVelocityInchesPerSec(), getRightVelocityInchesPerSec());
  robotstate.addObservations(time, odometry, velocity);
  left_encoder_prev_distance_ = left_distance;
  right_encoder_prev_distance_ = right_distance;
}
public void normalizeEncoders() {
  left_encoder_prev_distance_ = getLeftDistanceInches();
  right_encoder_prev_distance_ = getRightDistanceInches();
}
public boolean isFinishedPath() {
  return pathFollowingController_.isDone();
}
private double rotationsToInches(double rotations) {
  return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
}
private double rpmToInchesPerSecond(double rpm) {
  return rotationsToInches(rpm) / 60;
}
private double inchesPerSecondToVelo(double inches_per_second) {
  return inches_per_second * Constants.kRatioFactor;
}
public void zeroYaw() {
  ahrs.zeroYaw();
}
public void resetGyro() {
  ahrs.reset();
}
public double getYaw() {
  return ahrs.getYaw();
}
public double getAngle() {
  return ahrs.getAngle();
}
public void EnableVoltComp() {
  mDrive_Left_Master.enableVoltageCompensation(true);
  mDrive_Left_B.enableVoltageCompensation(true);
  mDrive_Left_C.enableVoltageCompensation(true);
  mDrive_Right_Master.enableVoltageCompensation(true);
  mDrive_Right_B.enableVoltageCompensation(true);
  mDrive_Right_C.enableVoltageCompensation(true);
}
public void DisableVoltComp() {
  mDrive_Left_Master.enableVoltageCompensation(false);
  mDrive_Left_B.enableVoltageCompensation(false);
  mDrive_Left_C.enableVoltageCompensation(false);
  mDrive_Right_Master.enableVoltageCompensation(false);
  mDrive_Right_B.enableVoltageCompensation(false);
  mDrive_Right_C.enableVoltageCompensation(false);
}
public void StopDrivetrain() {
  mDrive_Left_Master.set(ControlMode.PercentOutput, 0.0);
  mDrive_Right_Master.set(ControlMode.PercentOutput, 0.0);
}
public void pidTurn() {
  leftpow_ = -PIDturnOutput.getOutput();
  rightpow_ = PIDturnOutput.getOutput();
  tank(leftpow_, rightpow_);
  }
public double getTurnOutput() {
    return PIDturnOutput.getOutput();
}
public void setBrake() {
  mDrive_Left_Master.setNeutralMode(NeutralMode.Brake);
  mDrive_Left_B.setNeutralMode(NeutralMode.Brake);
  mDrive_Left_C.setNeutralMode(NeutralMode.Brake);
  mDrive_Right_Master.setNeutralMode(NeutralMode.Brake);
  mDrive_Right_B.setNeutralMode(NeutralMode.Brake);
  mDrive_Right_C.setNeutralMode(NeutralMode.Brake);
}
public void setCoast() {
  mDrive_Left_Master.setNeutralMode(NeutralMode.Coast);
  mDrive_Left_B.setNeutralMode(NeutralMode.Coast);
  mDrive_Left_C.setNeutralMode(NeutralMode.Coast);
  mDrive_Right_Master.setNeutralMode(NeutralMode.Coast);
  mDrive_Right_B.setNeutralMode(NeutralMode.Coast);
  mDrive_Right_C.setNeutralMode(NeutralMode.Coast);
}
public void resetEncoders() {
  mDrive_Left_Master.setSelectedSensorPosition(0, 0, Constants.kTimeoutms);
  mDrive_Right_Master.setSelectedSensorPosition(0, 0, Constants.kTimeoutms);
}
public void ReportData() {
  SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
  SmartDashboard.putNumber("turnoutput", getTurnOutput());
  SmartDashboard.putNumber("leftIPS", getLeftVelocityInchesPerSec());
  SmartDashboard.putNumber("rightIPS", getRightVelocityInchesPerSec());
  SmartDashboard.putNumber("leftraw", mDrive_Left_Master.getSelectedSensorPosition());
  SmartDashboard.putNumber("rightraw", mDrive_Right_Master.getSelectedSensorPosition());
}
  /* Returns distance in inches */
public double getLeftDistanceInches() {
  return rotationsToInches(mDrive_Left_Master.getSelectedSensorPosition()/39321.6);
}
public double getRightDistanceInches() {
  return rotationsToInches(mDrive_Left_Master.getSelectedSensorPosition()/39321.6);
}
public double getLeftVelocityInchesPerSec() {
  return rpmToInchesPerSecond(mDrive_Left_Master.getSelectedSensorVelocity()*10/39321.6*60);
}
public double getRightVelocityInchesPerSec() {
  return rpmToInchesPerSecond(mDrive_Right_Master.getSelectedSensorVelocity()*10/39321.6*60);
}
public void resetPosition() {
  resetEncoders();
  ahrs.reset();
}
@Override
public void initDefaultCommand() {
}
}
