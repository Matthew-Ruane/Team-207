package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.utility.*;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.OI;
/**
 * Author Matt Ruane (207)
 */
public class Drivebase extends Subsystem {

  private static final Drivebase instance = new Drivebase();

  public static Drivebase getInstance() {
    return instance;
  }

  private static final DefaultDriveTalonSRX mDrive_Left_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
  private static final DefaultDriveTalonSRX mDrive_Left_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
  private static final DefaultDriveTalonSRX mDrive_Left_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);
  private static final DefaultDriveTalonSRX mDrive_Right_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
  private static final DefaultDriveTalonSRX mDrive_Right_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
  private static final DefaultDriveTalonSRX mDrive_Right_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);

  public static SpeedControllerGroup mDrive_Left = new SpeedControllerGroup(mDrive_Left_Master, mDrive_Left_B, mDrive_Left_C);
  public static SpeedControllerGroup mDrive_Right = new SpeedControllerGroup(mDrive_Right_Master, mDrive_Right_B, mDrive_Right_C);

  public static DifferentialDrive mDrive;

  private static Solenoid mShifter_High = new Solenoid(RobotMap.mPCM_B, RobotMap.mShift_High_ID);
  private static Solenoid mShifter_Low = new Solenoid(RobotMap.mPCM_A, RobotMap.mShift_Low_ID);

  public static final Encoder rightEncoder = new Encoder(1, 2, false, EncodingType.k4X);
  public static final Encoder leftEncoder = new Encoder(3, 4, false, EncodingType.k4X);

  public void Drivebase() {
    
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
    mDrive.arcadeDrive(OI.getThrottleInput(), OI.getThrottleInputInverted());
  }
  public static void tank(double left, double right) {
    mDrive.tankDrive(left, right);
  }
  public static int getCurrentGear() {
    return Constants.CURRENT_GEAR;
  }
  public static double getleftEncoder() {
    return leftEncoder.getRaw();
  }
  public static double getrightEncoder() {
    return rightEncoder.getRaw();
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
  public static void initDrive() {
    mDrive_Left_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    mDrive_Right_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    mDrive.setRightSideInverted(true);
    mShifter_High.set(RobotMap.On);
    mShifter_Low.set(RobotMap.Off);
    mDrive_Left_B.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Left_C.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Right_B.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    mDrive_Right_C.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    resetEncoders();
    configEncoders();
  }

  @Override
  public void initDefaultCommand() {
  }
}
