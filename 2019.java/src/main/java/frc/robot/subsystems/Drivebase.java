package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Encoder;
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

  public static DifferentialDrive mDrive;

  private static Solenoid mShifter_High;
  private static Solenoid mShifter_Low;

  public static Encoder rightEncoder;
  public static Encoder leftEncoder;

  public static int leftEncoderZero = 0;
  public static int rightEncoderZero = 0;


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
    leftEncoder = new Encoder(3, 4, false, EncodingType.k4X);
    rightEncoder = new Encoder(1, 2, false, EncodingType.k4X);
    leftEncoder.setReverseDirection(true);
    rightEncoder.setReverseDirection(false);
    resetEncoders();
    configEncoders();
    
    mShifter_Low = new Solenoid(RobotMap.mPCM_A, RobotMap.mShift_Low_ID);
    mShifter_High = new Solenoid(RobotMap.mPCM_B, RobotMap.mShift_High_ID);
    mShifter_High.set(RobotMap.On);
    mShifter_Low.set(RobotMap.Off);
    Constants.CURRENT_GEAR = Constants.HIGH_GEAR;
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

  	/**
	 * Zeros the left encoder position in software
	 */
	public static void zeroLeftEncoder() {
    leftEncoderZero = leftEncoder.get();
	}

	/**
	 * Zeros the right encoder position in software
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
		return getLeftEncoderTicks()*RobotMap.wheel_distance_in_per_tick;
	}

	/**
	 * Get the distance traveled by right wheel, in inches ticks since last zeroRightEncoder()
	 * 
	 * @return distance traveled, in inches
	 */
	public static double getRightDistance() {
		return getRightEncoderTicks()*RobotMap.wheel_distance_in_per_tick;
	}
  	/**
	 * Set the percent output of the left motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public static void setLeftMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		mDrive_Left.set(-powerPct);
  }
  
	/**
	 * Set the percent output of the right motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public static void setRightMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		mDrive_Right.set(powerPct);
	}
  
  @Override
  public void initDefaultCommand() {
  }
}
