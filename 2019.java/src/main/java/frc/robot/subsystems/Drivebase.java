package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
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

/**
 * Author Matt Ruane (207)
 */
public class Drivebase extends Subsystem {

  private static final Drivebase instance = new Drivebase();

  public static Drivebase getInstance() {
    return instance;
  }

  public DefaultDriveTalonSRX mDrive_Left_Master, mDrive_Left_B, mDrive_Left_C, mDrive_Right_Master, mDrive_Right_B, mDrive_Right_C;

  public static SpeedControllerGroup mDrive_Left;
  public static SpeedControllerGroup mDrive_Right;

  private static double leftspeed, rightspeed;

  public static DifferentialDrive mDrive;

  private static Solenoid mShifter_High;
  private static Solenoid mShifter_Low;

  public static Encoder rightEncoder;
  public static Encoder leftEncoder;

  //private PIDController PIDturn, PIDleft, PIDright;

  private DummyPIDOutput PIDturnOutput;

  public static int leftEncoderZero = 0;
  public static int rightEncoderZero = 0;

  private static double x, y, distance, leftEncoderDistance, prevLeftEncoderDistance, rightEncoderDistance, prevRightEncoderDistance, gyroAngle;
  private static double setAngle = 0;

  private static double yawZero = 0;
  public static AHRS ahrs;

  private PIDController PIDturn, PIDleft, PIDright;

  public Drivebase() {
    mDrive_Left_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
    mDrive_Left_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
    mDrive_Left_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);
    mDrive_Right_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
    mDrive_Right_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
    mDrive_Right_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);
    mDrive_Left = new SpeedControllerGroup(mDrive_Left_Master, mDrive_Left_B, mDrive_Left_C);
    mDrive_Right = new SpeedControllerGroup(mDrive_Right_Master, mDrive_Right_B, mDrive_Right_C);
    
    mDrive = new DifferentialDrive(mDrive_Left, mDrive_Right);
    mDrive.setSafetyEnabled(false);
    leftEncoder = new Encoder(3, 4, false, EncodingType.k1X);
    rightEncoder = new Encoder(1, 2, false, EncodingType.k1X);
    leftEncoder.setReverseDirection(true);
    rightEncoder.setReverseDirection(false);
    resetEncoders();
    configEncoders();
    
    mShifter_Low = new Solenoid(RobotMap.mPCM_A, RobotMap.mShift_Low_ID);
    mShifter_High = new Solenoid(RobotMap.mPCM_B, RobotMap.mShift_High_ID);
    mShifter_High.set(RobotMap.On);
    mShifter_Low.set(RobotMap.Off);
    Constants.CURRENT_GEAR = Constants.HIGH_GEAR;

    ahrs = new AHRS(SerialPort.Port.kMXP);

    PIDturnOutput = new DummyPIDOutput();
    PIDturn = new PIDController(Constants.Turn_kP, Constants.Turn_kI, Constants.Turn_kD, ahrs, PIDturnOutput);

    PIDturn.setInputRange(-180.0f,  180.0f);
    PIDturn.setOutputRange(-1.0, 1.0);
    PIDturn.setAbsoluteTolerance(Constants.kToleranceDegrees);
    PIDturn.setContinuous(true);
  }
  
  public static void UpShift() {
    mShifter_High.set(RobotMap.On);
    mShifter_Low.set(RobotMap.Off);
    Constants.CURRENT_GEAR = Constants.HIGH_GEAR;
  }
  public static void DownShift() {
    mShifter_High.set(RobotMap.Off);
    mShifter_Low.set(RobotMap.On);
    Constants.CURRENT_GEAR = Constants.LOW_GEAR;
  }
  public static void arcade() {
    mDrive.arcadeDrive(OI.getThrottleInput(), OI.getSteeringInputInverted());
  }
  public static void tank(double left, double right) {
    mDrive.tankDrive(left, right);
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
      	/**
	 * Resets the gyro position in software to a specified angle
	 * 
	 * @param currentHeading Gyro heading to reset to, in degrees
	 */
	public static void setGyroRotation(double currentHeading) {
		// set yawZero to gyro angle, offset to currentHeading
    yawZero = -ahrs.getAngle() - currentHeading;
  }
  	/**
	 * Zeros the gyro position in software
	 */
	public static void zeroGyroRotation() {
		// set yawZero to gryo angle
    yawZero = -ahrs.getAngle();
  }
  public static double getGyroRotation() {
    double angle = -ahrs.getAngle() - yawZero;
    // Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
    // to (-180, 180]
    angle = angle % 360;
    angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
    return angle;
  }
  public void RotateToAngle() {
    PIDturn.setSetpoint(setAngle);
    tank(PIDturnOutput.getOutput(), -PIDturnOutput.getOutput());
    PIDturn.enable();
  }
  public void StopRotateToAngle() {
    PIDturn.disable();
    setAngle = 0;
  }
  public static void ReportData() {
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
  }
  /* 
  End NavX specific content */
  public static int getCurrentGear() {
    return Constants.CURRENT_GEAR;
  }
  public static int getleftEncoder() {
    return leftEncoder.get();
  }
  public static int getrightEncoder() {
    return rightEncoder.get();
  }
  public static void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
  public static void configEncoders() {
    leftEncoder.setDistancePerPulse(Constants.getWheelCircumference()/(4096*Constants.kEncoderDriveRatio));
    rightEncoder.setDistancePerPulse(Constants.getWheelCircumference()/(4096*Constants.kEncoderDriveRatio));
  }
  public static double getLeftPosition() {
		return MotionUtils.rotationsToDistance(MotionUtils.ticksToRotations(leftEncoder.getRaw(), 4096, Constants.kEncoderDriveRatio), Constants.getWheelCircumference());
	}
	public static double getRightPosition() {
		return MotionUtils.rotationsToDistance(MotionUtils.ticksToRotations(rightEncoder.getRaw(), 4096, Constants.kEncoderDriveRatio), Constants.getWheelCircumference());
  }
  public static double getLeftVelocity() {
		return leftEncoder.getRate();
	}
	public static double getRightVelocity() {
		return rightEncoder.getRate();
  }
  public void calcXY() {
		 leftEncoderDistance  = leftEncoder.getDistance();
		 rightEncoderDistance = rightEncoder.getDistance();
		 gyroAngle = getAngle();
		 distance =  ((leftEncoderDistance - prevLeftEncoderDistance) + (rightEncoderDistance - prevRightEncoderDistance))/2;
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

  	/**
	 * Captures left Encoder starting position for zeroing
	 */
	public static void zeroLeftEncoder() {
    leftEncoderZero = leftEncoder.get();
	}

	/**
	 * Captures right Encoder starting position for zeroing
	 */
	public static void zeroRightEncoder() {
		rightEncoderZero = rightEncoder.get();
	}

	/**
	 * Get the position of the left encoder, in encoder ticks since last zeroLeftEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public static int getLeftEncoderTicks() {
		return leftEncoder.get() - leftEncoderZero;
	}
	/**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public static int getRightEncoderTicks() {
		return rightEncoder.get() + rightEncoderZero;
	}
	/**
	 * Get the distance traveled by left wheel, in inches since last zeroLeftEncoder()
	 * 
	 * @return distance traveled, in inches
	 */
	public static double getLeftDistance() {
		return getLeftEncoderTicks()*Constants.wheel_distance_in_per_tick;
	}
	/**
	 * Get the distance traveled by right wheel, in inches ticks since last zeroRightEncoder()
	 * 
	 * @return distance traveled, in inches
	 */
	public static double getRightDistance() {
		return getRightEncoderTicks()*Constants.wheel_distance_in_per_tick;
	}
  	/**
	 * Set the percent output of the left motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public static void setLeftMotors(double powerPct) {
		mDrive_Left.set(-powerPct);
  }
	/**
	 * Set the percent output of the right motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public static void setRightMotors(double powerPct) {
		mDrive_Right.set(powerPct);
	}
  
  @Override
  public void initDefaultCommand() {
  }
}
