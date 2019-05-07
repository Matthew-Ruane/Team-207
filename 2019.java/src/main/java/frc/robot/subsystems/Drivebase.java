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

  public static SpeedControllerGroup mDrive_Left;
  public static SpeedControllerGroup mDrive_Right;

  private static double leftspeed, rightspeed;

  public static DifferentialDrive mDrive;

  private static Solenoid mShifter_High, mShifter_Low;

  public static Encoder rightEncoder, leftEncoder;

  private static DummyPIDOutput PIDturnOutput, PIDleftOutput, PIDrightOutput;

  public static int leftEncoderZero = 0;
  public static int rightEncoderZero = 0;

  private static double x, y, distance, leftEncoderDistance, prevLeftEncoderDistance, rightEncoderDistance, 
                        prevRightEncoderDistance, gyroAngle, desiredDistanceInches, desiredDistanceTicks,
                        TurnrateCurved;

  private static double setAngle = 0;
  private static double desiredAngle = 0;


  private static double yawZero = 0;
  public static AHRS ahrs;

  public static PIDController PIDturn, PIDleft, PIDright;


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
    mShifter_High.set(Constants.On);
    mShifter_Low.set(Constants.Off);
    Constants.CURRENT_GEAR = Constants.HIGH_GEAR;

    ahrs = new AHRS(SerialPort.Port.kMXP);

    PIDturnOutput = new DummyPIDOutput();
    PIDleftOutput =  new DummyPIDOutput();
    PIDrightOutput = new DummyPIDOutput();

    PIDturn = new PIDController(Constants.Turn_kP, Constants.Turn_kI, Constants.Turn_kD, ahrs, PIDturnOutput);
    PIDleft = new PIDController(Constants.Drive_kP, Constants.Drive_kI, Constants.Drive_kD, Constants.Drive_kD, leftEncoder, PIDleftOutput);
    PIDright = new PIDController(Constants.Drive_kP, Constants.Drive_kI, Constants.Drive_kD, Constants.Drive_kD, rightEncoder, PIDrightOutput);

    PIDturn.setInputRange(-180.0,  180.0);
    PIDturn.setOutputRange(-0.65, 0.65);
    PIDturn.setAbsoluteTolerance(Constants.kToleranceDegrees);
    PIDturn.setContinuous(true);

    PIDleft.setAbsoluteTolerance(Constants.kToleranceDistance);
    PIDleft.setInputRange(-1.0, 1.0);
    PIDleft.setOutputRange(-1.0, 1.0);
    PIDleft.setContinuous(true);

    PIDright.setAbsoluteTolerance(Constants.kToleranceDistance);
    PIDright.setInputRange(-1.0, 1.0);
    PIDright.setOutputRange(-1.0, 1.0);
    PIDright.setContinuous(true);
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
  public static void arcade() {
    mDrive.arcadeDrive(OI.getLeftThrottleInput(), OI.getRightSteeringInputInverted());
  }
  public static void tank(double left, double right) {
    mDrive.tankDrive(left, right);
  }
  public static void curvature() {
    TurnrateCurved = (Constants.kTurnrateCurve * Math.pow(OI.getLeftSteeringInputInverted(), 3)+(1-Constants.kTurnrateCurve) * OI.getLeftSteeringInputInverted() * Constants.kTurnrateLimit);
    mDrive.curvatureDrive(OI.getRightThrottleInput(), TurnrateCurved, true);
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
      	/**
	 * Resets the gyro position in software to a specified angle
	 * 
	 /*
	 @param currentHeading Gyro heading to reset to, in degrees*/
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
  /* Methods for locking heading and drive to setpoints.  Tuning ongoing as of 5/3/19.  uses xxx.enable() and xxx.disable to start.  Config set in class constructor. */
  public static void RotateToAngle(double desiredAngle) {
    PIDturn.setSetpoint(desiredAngle);
    tank(-PIDturnOutput.getOutput(), PIDturnOutput.getOutput());
  }
  public void StopRotateToAngle() {
    PIDturn.disable();
  }
  public static void pidDrive() {
    mDrive_Left.set(-PIDleftOutput.getOutput()-PIDturnOutput.getOutput());
    mDrive_Right.set(-PIDrightOutput.getOutput()+PIDturnOutput.getOutput());
  }
  public static void setDriveDistance (double desiredDistanceInches) {
    desiredDistanceTicks = desiredDistanceInches*347.22;
    PIDleft.setSetpoint(getLeftEncoderTicks()+desiredDistanceTicks);
    PIDright.setSetpoint(getRightEncoderTicks()+desiredDistanceTicks);
  }
  public static void pidDrive_Disable() {
    PIDleft.disable();
    PIDright.disable();
  }
  public static void pidDrive_Enable() {
    PIDleft.enable();
    PIDright.enable();
  }
  public static void ReportData() {
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
  }
  /* End NavX specific content */
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
  /* Joe Ross' X-Y plane position tracking */
  public void calcXY() {
		 leftEncoderDistance  = leftEncoder.getDistance();
		 rightEncoderDistance = rightEncoder.getDistance();
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
/* 294's methods for zeroing encoders before use in drive PID loops */
	public static void zeroLeftEncoder() {
    leftEncoderZero = leftEncoder.get();
	}
	public static void zeroRightEncoder() {
		rightEncoderZero = rightEncoder.get();
	}
	public static int getLeftEncoderTicks() {
		return leftEncoder.get() - leftEncoderZero;
	}
	public static int getRightEncoderTicks() {
		return rightEncoder.get() + rightEncoderZero;
	}
	public static double getLeftDistance() {
		return getLeftEncoderTicks()*Constants.encoderTicksPerInch;
	}
	public static double getRightDistance() {
		return getRightEncoderTicks()*Constants.encoderTicksPerInch;
	}

  @Override
  public void initDefaultCommand() {
  }
}
