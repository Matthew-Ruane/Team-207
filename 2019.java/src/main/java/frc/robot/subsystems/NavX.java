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
    private static double yawZero = 0;
    public static AHRS ahrs;

    private static final NavX instance = new NavX();

    public static NavX getInstance() {
      return instance;
    }
    public NavX() {
      ahrs = new AHRS(SerialPort.Port.kMXP);
    }

    public static void zeroYaw() {
      ahrs.zeroYaw();
    }
    public static double getYaw() {
      return ahrs.getYaw();
    }
      	/**
	 * Resets the gyro position in software to a specified angle
	 * 
	 * @param currentHeading Gyro heading to reset to, in degrees
	 */
	public static void setGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
    yawZero = -ahrs.getAngle() - currentHeading;
  }
    // System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
  	/**
	 * Zeros the gyro position in software
	 */
	public static void zeroGyroRotation() {
		// set yawZero to gryo angle
    yawZero = -ahrs.getAngle();
  }
    // System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
    public static double getGyroRotation() {
      double angle = -ahrs.getAngle() - yawZero;
      // Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
      // to (-180, 180]
      angle = angle % 360;
      angle = (angle <= -180) ? (angle + 360) : angle;
      angle = (angle > 180) ? (angle - 360) : angle;
      return angle;
    }
    public static void ReportData() {
      SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
      SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
      SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
      SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
      SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    }
}
