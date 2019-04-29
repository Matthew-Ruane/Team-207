package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.drive.*;
import frc.utility.DefaultDriveTalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;


/**
 * Author Matt Ruane (207)
 */
public class Drivebase extends Subsystem {

  private static final Drivebase instance = new Drivebase();

  public static Drivebase getInstance() {
    return instance;
  }

  public static final AHRS ahrs = new AHRS(I2C.Port.kMXP);
  public static final ADXRS450_Gyro kGyro = new ADXRS450_Gyro(Port.kMXP);

  private static final DefaultDriveTalonSRX mDrive_Left_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
  private static final DefaultDriveTalonSRX mDrive_Left_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
  private static final DefaultDriveTalonSRX mDrive_Left_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);
  private static final DefaultDriveTalonSRX mDrive_Right_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
  private static final DefaultDriveTalonSRX mDrive_Right_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
  private static final DefaultDriveTalonSRX mDrive_Right_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);

  public static DifferentialDrive mDrive = new DifferentialDrive(mDrive_Left_Master, mDrive_Right_Master);

  private static Solenoid mShifter_High = new Solenoid(RobotMap.mPCM_A, RobotMap.mShift_High_ID);
  private static Solenoid mShifter_Low = new Solenoid(RobotMap.mPCM_B, RobotMap.mShift_Low_ID);

  private void Drivebase() {

    mDrive_Left_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
    mDrive_Right_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);

    mDrive_Left_B.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Left_C.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Right_B.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    mDrive_Right_C.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);

    mDrive.setRightSideInverted(false);
    mShifter_High.set(RobotMap.On);
    mShifter_Low.set(RobotMap.Off);
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
  public static int getCurrentGear() {
    return Constants.CURRENT_GEAR;
  }
  public static void CalibrateGyro() {
    kGyro.calibrate();
  }
  public static void ResetGyro() {
    kGyro.reset();
  }
  public static double getHeading() {
    return kGyro.getAngle();
  }
  public static double getTurnRate() {
    return kGyro.getRate();
  }

  @Override
  public void initDefaultCommand() {
  }
}
