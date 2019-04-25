/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.*;
import frc.utility.DefaultDriveTalonSRX;
import frc.utility.DefaultDriveTalonSRX;
import frc.robot.OI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;


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

  public static int HIGH_GEAR = 0;
  public static int LOW_GEAR = 1;
  public static int CURRENT_GEAR = HIGH_GEAR;

  private static void Drivebase() {

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
    CURRENT_GEAR = HIGH_GEAR;
  }
  public static void DownShift() {
    mShifter_High.set(RobotMap.Off);
    mShifter_Low.set(RobotMap.On);
    CURRENT_GEAR = LOW_GEAR;
  }
  public static int getCurrentGear() {
    return CURRENT_GEAR;
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

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
