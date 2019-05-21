package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.DriveWithJoysticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team294.utilities.*;

/**
 * A Subsystem to control the Drive Train
 */
public class DriveTrain extends Subsystem {
	private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightMotor1);
	private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightMotor2);
	private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RobotMap.rightMotor3);
	private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(RobotMap.leftMotor1);
	private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftMotor2);
	private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(RobotMap.leftMotor3);
	public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2); 

	// Gyro variables
	private AHRS ahrs;
	private double yawZero = 0;			// Track gyro zero here to minimize latency in CanBus

	// Timer for logging motor data
	private int periodicCount = 0;
	
	private double fieldX;
	private double fieldY;

	// Track left and right encoder zeros here to minimize latency in Talons.
	private double leftEncoderZero = 0, rightEncoderZero = 0;

	public DriveTrain() {// initialize Followers
		// motor2 are the main motors, and motor 1 and 3 are the followers.
		// Robot.driveDirection is a value pulled from Robot Preferences.
		// True means that it will drive forward correctly, and false drives it
		// backwards.
		// This is used when testing drivetrain code on the 2017 practice base.
		// reads robot preferences to see if it is prototype. The competition robot has
		// an encoder that is in reverse of the prototype robot.

		leftMotor2.setInverted(Robot.robotPrefs.driveDirection);
		rightMotor2.setInverted(Robot.robotPrefs.driveDirection);
		leftMotor1.setInverted(Robot.robotPrefs.driveDirection);
		rightMotor1.setInverted(Robot.robotPrefs.driveDirection);
		leftMotor3.setInverted(Robot.robotPrefs.driveDirection);
		rightMotor3.setInverted(Robot.robotPrefs.driveDirection);

		leftMotor2.setSensorPhase(true);

		leftMotor1.set(ControlMode.Follower, RobotMap.leftMotor2);
		leftMotor3.set(ControlMode.Follower, RobotMap.leftMotor2);
		rightMotor1.set(ControlMode.Follower, RobotMap.rightMotor2);
		rightMotor3.set(ControlMode.Follower, RobotMap.rightMotor2);
		leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		// Enable voltage compensation on drive motors (to account for battery voltage
		// droop)
		leftMotor2.enableVoltageCompensation(true);
		rightMotor2.enableVoltageCompensation(true);
		leftMotor2.configVoltageCompSaturation(11.0, 0);
		rightMotor2.configVoltageCompSaturation(11.0, 0);

		SmartDashboard.putBoolean("DDirection", Robot.robotPrefs.driveDirection);
		System.out.println("Drive Direction: " + Robot.robotPrefs.driveDirection);

		leftMotor2.clearStickyFaults(0);
		rightMotor2.clearStickyFaults(0);

		leftMotor2.setNeutralMode(NeutralMode.Brake);
		rightMotor2.setNeutralMode(NeutralMode.Brake);

		zeroLeftEncoder();
		zeroRightEncoder();
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
		ahrs.zeroYaw();
	}

	public void setFieldPositionX(double x) {
		this.fieldX = x;
		SmartDashboard.putNumber("FieldX", fieldX);
	}

	public void setFieldPositionY(double y) {
		this.fieldY = y;
		SmartDashboard.putNumber("FieldY", fieldY);
	}

	public void addFieldPositionX(double x) {
		setFieldPositionX(fieldX + x);
	}

	public void addFieldPositionY(double y) {
		setFieldPositionY(fieldY + y);
	}

	public double getFieldPositionX() {
		return this.fieldX;
	}

	public double getFieldPositionY() {
		return this.fieldY;
	}

	/**
	 * Zeros the gyro position in software
	 */
	public void zeroGyroRotation() {
		// set yawZero to gryo angle
		yawZero = ahrs.getAngle();
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Gets the rotation of the gyro
	 * 
	 * @return Current angle from 0 to 360 degrees
	 */
	public double getGyroRotation() {
		double angle = ahrs.getAngle() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
		// to [0,360]
		angle = angle % 360;
		SmartDashboard.putNumber("NavX Angle", angle);
		return angle;
	}

	public void tankDrive(double powerLeft, double powerRight) {
		this.robotDrive.tankDrive(powerLeft, powerRight);
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
	 * Get the position of the left encoder, in encoder ticks
	 * 
	 * @return encoder position, in ticks
	 */
	public double getLeftEncoderTicks() {
		double position;
		if (!Robot.robotPrefs.prototypeRobot) {
			position = -leftMotor2.getSelectedSensorPosition(0) + leftEncoderZero;
		} else {
			position = leftMotor2.getSelectedSensorPosition(0) - leftEncoderZero;
		}
		return position;
	}

	/**
	 * Get the position of the right encoder, in encoder ticks
	 * 
	 * @return encoder position, in ticks
	 */
	public double getRightEncoderTicks() {
		double position;
		if (!Robot.robotPrefs.prototypeRobot) {
			position = -rightMotor2.getSelectedSensorPosition(0) + rightEncoderZero;
		} else {
			position = rightMotor2.getSelectedSensorPosition(0) - rightEncoderZero;
		}
		return position;
	}

	/**
	 * Converts drive encoder ticks to inches
	 * 
	 * @param encoderticks
	 *            distance, in ticks
	 * @return distance, in inches
	 */
	public double encoderTicksToInches(double encoderticks) {
		return (encoderticks / RobotMap.encoderTicksPerRevolution) * Robot.robotPrefs.wheelCircumference
				* Robot.robotPrefs.driveTrainDistanceFudgeFactor;
	}

	/**
	 * Converts inches to drive encoder ticks
	 * 
	 * @param inches
	 *            distance, in inches
	 * @return distance, in ticks
	 */
	public double inchesToEncoderTicks(double inches) {
		return (inches / Robot.robotPrefs.wheelCircumference / Robot.robotPrefs.driveTrainDistanceFudgeFactor)
				* RobotMap.encoderTicksPerRevolution;
	}

	/**
	 * Get the position of the left encoder, in inches
	 * 
	 * @return encoder position, in inches
	 */
	public double getLeftEncoderInches() {
		double inches = encoderTicksToInches(getLeftEncoderTicks());
		SmartDashboard.putNumber("Left Encoder Position", inches);
		return inches;
	}

	/**
	 * Get the position of the right encoder, in inches
	 * 
	 * @return encoder position, in inches
	 */
	public double getRightEncoderInches() {
		double inches = encoderTicksToInches(getRightEncoderTicks());
		SmartDashboard.putNumber("Right Encoder Position", inches);
		return inches;
	}

	/**
	 * Get the average position of the two encoders, in inches
	 * 
	 * @return encoder position, in inches
	 */
	public double getAverageEncoderInches() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
	}

	/**
	 * Sets the robot to drive at a curve.
	 * 
	 * @param speedPct
	 *            Percent output of motor -1.0 to 1.0
	 * @param curve
	 *            the rate at which the robot will curve -1.0 to 1.0. Clockwise is
	 *            positive.
	 */
	public void driveAtCurve(double speedPct, double curve) {
		robotDrive.curvatureDrive(speedPct, curve, false);
	}

	/**
	 * Set the percent output of the left motor.
	 * 
	 * @param powerPct
	 *            Percent of power -1.0 to 1.0
	 */
	public void setLeftMotors(double speedPct) {
		leftMotor2.set(ControlMode.PercentOutput, -speedPct);
	}

	/**
	 * Set the percent output of the right motor.
	 * 
	 * @param powerPct
	 *            Percent of power -1.0 to 1.0
	 */
	public void setRightMotors(double speedPct) {
		rightMotor2.set(ControlMode.PercentOutput, speedPct);
	}

	/**
	 * Stops all motors.
	 */
	public void stopAllMotors() {
		setLeftMotors(0);
		setRightMotors(0);
	}

	// Periodic is called once every scheduler cycle (20ms)
	public void periodic() {
		// Display info on Dashboard
		// getGyroRotation();
		getLeftEncoderInches();
		getRightEncoderInches();

		if (DriverStation.getInstance().isEnabled()) {
			// Log every 50 cycles (~1 second)
			if ( (++periodicCount)>=50 ) {
				updateDriveLog();
				periodicCount = 0;
			}
		}
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
	}

	public void updateDriveLog() {
		Robot.log.writeLog("Left Motor 1 Output Voltage," + leftMotor1.getMotorOutputVoltage()
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
				+ rightMotor3.getMotorOutputPercent() + ",Left Encoder Inches," + getLeftEncoderInches()
				+ ",Left Encoder Ticks," + getLeftEncoderInches() + ",Right Encoder Inches," + getRightEncoderInches()
				+ ",Right Encoder Ticks," + getRightEncoderTicks() + ",Average Encoder Inches,"
				+ getAverageEncoderInches() + ",Gyro Rotation," + getGyroRotation() + ",Field Position X,"
				+ getFieldPositionX() + ",Field Position Y," + getFieldPositionY() + ",High Gear,"
				+ Robot.shifter.isShifterInHighGear());
	}
}
