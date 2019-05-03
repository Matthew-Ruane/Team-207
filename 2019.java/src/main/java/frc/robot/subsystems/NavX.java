package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
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
  public static void ReportData() {
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
  }
}
