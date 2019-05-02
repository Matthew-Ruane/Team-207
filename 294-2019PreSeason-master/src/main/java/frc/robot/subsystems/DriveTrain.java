/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

	// Motors
	private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(RobotMap.leftDrive1);
	private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftDrive2);
	private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(RobotMap.leftDrive3);

	private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightDrive1);
	private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightDrive2);
	private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RobotMap.rightDrive3);
	public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);

	// Gyro variables
	private AHRS ahrs;
	private double yawZero = 0;

	// Encoder software zeros
	private int leftEncoderZero = 0, rightEncoderZero = 0;

	// FileLog timer
	private long lastLogTimeMillis = 0;

	// Max motor voltage (used for voltage compensation)
	private static double MAX_VOLTAGE = 11.0;

	public DriveTrain() {
		// Set motor2 on each side as master controllers
		leftMotor1.set(ControlMode.Follower, RobotMap.leftDrive2);
		leftMotor3.set(ControlMode.Follower, RobotMap.leftDrive2);
		rightMotor1.set(ControlMode.Follower, RobotMap.rightDrive2);
		rightMotor3.set(ControlMode.Follower, RobotMap.rightDrive2);

		// Invert motor poloarity, so + = forward, - = reverse
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		leftMotor3.setInverted(true);
		rightMotor1.setInverted(true);
		rightMotor2.setInverted(true);
		rightMotor3.setInverted(true);

		// Configure encoders on master controllers
		leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		zeroLeftEncoder();
		zeroRightEncoder();

		// Set Netrual mode to coast to ease wear and tear on gearbox
		leftMotor1.setNeutralMode(NeutralMode.Coast);
		leftMotor2.setNeutralMode(NeutralMode.Coast);
		leftMotor3.setNeutralMode(NeutralMode.Coast);
		rightMotor1.setNeutralMode(NeutralMode.Coast);
		rightMotor2.setNeutralMode(NeutralMode.Coast);
		rightMotor3.setNeutralMode(NeutralMode.Coast);

		// Configure motors to voltage compensation mode (but don't enable automatically)
		// Enable voltage compensation using setVoltageCompsation(true);
		// Disable voltage compensation using setVoltageCompsation(false);
		leftMotor1.configVoltageCompSaturation(MAX_VOLTAGE, 0);
		leftMotor2.configVoltageCompSaturation(MAX_VOLTAGE, 0);
		leftMotor3.configVoltageCompSaturation(MAX_VOLTAGE, 0);
		rightMotor1.configVoltageCompSaturation(MAX_VOLTAGE, 0);
		rightMotor2.configVoltageCompSaturation(MAX_VOLTAGE, 0);
		rightMotor3.configVoltageCompSaturation(MAX_VOLTAGE, 0);

		// Configure navX
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus. */
			/* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
			/*
			 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
			 * details.
			 */

			ahrs = new AHRS(I2C.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		// ahrs.zeroYaw();
		zeroGyroRotation();
	}

   /**
   * Tank drive method for differential drive platform.
   * The calculated values will be squared to decrease sensitivity at low speeds.
   * This is most useful for joystick driving, but not for autonomous code.
   *
   * @param powerLeft  The robot's left side speed along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   * @param powerRight The robot's right side speed along the X axis [-1.0..1.0]. Forward is
   *                   positive.
   */
	public void tankDrive(double powerLeft, double powerRight) {
		robotDrive.tankDrive(-powerLeft, -powerRight);
		// Robot.log.writeLog("DriveTrain", "tankDrive", "left power," + powerLeft + ",right power," + powerRight);
	}

	/**
	 * Set the percent output of the left motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public void setLeftMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		leftMotor2.set(ControlMode.PercentOutput, -powerPct);
	}

	/**
	 * Set the percent output of the right motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public void setRightMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		rightMotor2.set(ControlMode.PercentOutput, powerPct);
	}

	/**
	 * Stops all motors.
	 */
	public void stopAllMotors() {
		setLeftMotors(0);
		setRightMotors(0);
	}

	/**
	 * Turns voltage compensation on or off for drive motors.
	 * Voltage compensation increases accuracy for autonomous code,
	 * but it decreases maximum velocity/power when driving by joystick.
	 * @param turnOn true=turn on, false= turn off
	 */
	public void setVoltageCompensation(boolean turnOn) {
		leftMotor1.enableVoltageCompensation(turnOn);
		leftMotor2.enableVoltageCompensation(turnOn);
		leftMotor3.enableVoltageCompensation(turnOn);
		rightMotor1.enableVoltageCompensation(turnOn);
		rightMotor2.enableVoltageCompensation(turnOn);
		rightMotor3.enableVoltageCompensation(turnOn);
	}

	/**
	 * Zeros the left encoder position in software
	 */
	public void zeroLeftEncoder() {
		leftEncoderZero = leftMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Zeros the right encoder position in software
	 */
	public void zeroRightEncoder() {
		rightEncoderZero = rightMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Get the position of the left encoder, in encoder ticks since last zeroLeftEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public int getLeftEncoderTicks() {
		return leftMotor2.getSelectedSensorPosition(0) - leftEncoderZero;
	}

	/**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public int getRightEncoderTicks() {
		return -rightMotor2.getSelectedSensorPosition(0) + rightEncoderZero;
	}

	/**
	 * Get the distance traveled by left wheel, in inches since last zeroLeftEncoder()
	 * 
	 * @return distance traveled, in inches
	 */
	public double getLeftDistance() {
		return getLeftEncoderTicks()*RobotMap.wheel_distance_in_per_tick;
	}

	/**
	 * Get the distance traveled by right wheel, in inches ticks since last zeroRightEncoder()
	 * 
	 * @return distance traveled, in inches
	 */
	public double getRightDistance() {
		return getRightEncoderTicks()*RobotMap.wheel_distance_in_per_tick;
	}

	/**
	 * Get the average position of the two encoders, in inches
	 * 
	 * @return encoder position, in inches
	 */
	public double getAverageDistance() {
		return (getRightDistance() + getLeftDistance()) / 2.0;
	}

	/**
	 * Zeros the gyro position in software
	 */
	public void zeroGyroRotation() {
		// set yawZero to gryo angle
		yawZero = -ahrs.getAngle();
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Resets the gyro position in software to a specified angle
	 * 
	 * @param currentHeading Gyro heading to reset to, in degrees
	 */
	public void setGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = -ahrs.getAngle() - currentHeading;
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Gets the rotation of the gyro
	 * 
	 * @return Current angle from -180 to 180 degrees
	 */
	public double getGyroRotation() {
		double angle = -ahrs.getAngle() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
		// to (-180, 180]
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
		angle = (angle > 180) ? (angle - 360) : angle;
		return angle;
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveWithJoysticks());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Left Motor 1 current", leftMotor1.getOutputCurrent());
		SmartDashboard.putNumber("Left Motor 2 current", leftMotor2.getOutputCurrent());
		SmartDashboard.putNumber("Left Motor 3 current", leftMotor3.getOutputCurrent());
		SmartDashboard.putNumber("Right Motor 1 current", rightMotor1.getOutputCurrent());
		SmartDashboard.putNumber("Right Motor 2 current", rightMotor2.getOutputCurrent());
		SmartDashboard.putNumber("Right Motor 3 current", rightMotor3.getOutputCurrent());
		SmartDashboard.putNumber("Left encoder inches", getLeftDistance());
		SmartDashboard.putNumber("Right encoder inches", getRightDistance());
		SmartDashboard.putNumber("NavX Angle", getGyroRotation());

		// Log drivetrain information once per second, to keep an eye on the motors
		if (System.currentTimeMillis()-lastLogTimeMillis > 1000) {
			lastLogTimeMillis = System.currentTimeMillis();

			Robot.log.writeLog("DriveTrain", "periodic", 
			  "Left Motor 1 Output Voltage," + leftMotor1.getMotorOutputVoltage()
			+ ",Left Motor 1 Output Current," + leftMotor1.getOutputCurrent() + ",Left Motor 1 Output Percent,"
			+ leftMotor1.getMotorOutputPercent() + ",Left Motor 2 Output Voltage,"
			+ leftMotor2.getMotorOutputVoltage() + ",Left Motor 2 Output Current," + leftMotor2.getOutputCurrent()
			+ ",Left Motor 2 Output Percent," + leftMotor2.getMotorOutputPercent() + ",Left Motor 3 Output Voltage,"
			+ leftMotor3.getMotorOutputVoltage() + ",Left Motor 3 Output Current," + leftMotor3.getOutputCurrent()
			+ ",Left Motor 3 Output Percent," + leftMotor3.getMotorOutputPercent()
			+ ",Right Motor 1 Output Voltage," + rightMotor1.getMotorOutputVoltage()
			+ ",Right Motor 1 Output Current," + rightMotor1.getOutputCurrent() + ",Right Motor 1 Output Percent,"
			+ rightMotor1.getMotorOutputPercent() + ",Right Motor 2 Output Voltage,"
			+ rightMotor2.getMotorOutputVoltage() + ",Right Motor 2 Output Current,"
			+ rightMotor2.getOutputCurrent() + ",Right Motor 2 Output Percent,"
			+ rightMotor2.getMotorOutputPercent() + ",Right Motor 3 Output Voltage,"
			+ rightMotor3.getMotorOutputVoltage() + ",Right Motor 3 Output Current,"
			+ rightMotor3.getOutputCurrent() + ",Right Motor 3 Output Percent,"
			+ rightMotor3.getMotorOutputPercent() + ",Left Encoder Inches," + getLeftDistance()
			+ ",Left Encoder Ticks," + getLeftEncoderTicks() + ",Right Encoder Inches," + getRightDistance()
			+ ",Right Encoder Ticks," + getRightEncoderTicks() + ",Average Encoder Inches,"
			+ getAverageDistance() + ",Gyro Rotation," + getGyroRotation() 
			+ ",High Gear," + Robot.shifter.isShifterInHighGear());
		}
	}
}
