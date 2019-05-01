/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class NavX {
    public static final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

    private static final NavX instance = new NavX();

    public static NavX getInstance() {
      return instance;
    }
    private void NavX() {
    }

    public static void zeroYaw() {
      ahrs.zeroYaw();
    }
    public static double getAngle() {
      return ahrs.getAngle();
    }
    public static void ReportData() {
      SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
      SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
      SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
      SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
      SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    }
}
