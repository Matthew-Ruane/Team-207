package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ArmMotorSetToZero;
import org.usfirst.frc.team294.robot.commands.ClawSetMotorSpeed;
import org.usfirst.frc.team294.robot.triggers.MotorCurrentTrigger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Claw extends Subsystem {

	private final DigitalInput bumpSwitch = new DigitalInput(RobotMap.bumpSwitchClaw);
	private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchClaw);
	private final Solenoid clawPiston = new Solenoid(RobotMap.pneumaticClawPistonOut);
	private boolean bumpSwitchOverride;
	
	private final TalonSRX clawMotorLeft = new TalonSRX(RobotMap.clawMotorLeft);
	private final TalonSRX clawMotorRight = new TalonSRX(RobotMap.clawMotorRight);
	
	public final MotorCurrentTrigger clawMotorLeftCurrentTrigger = new MotorCurrentTrigger(clawMotorLeft, 8, 4);
	public final MotorCurrentTrigger clawMotorRightCurrentTrigger = new MotorCurrentTrigger(clawMotorRight, 8, 4);
	

	public Claw() {
		// Configure talons
		clawMotorLeft.set(ControlMode.PercentOutput, 0);
		clawMotorLeft.setNeutralMode(NeutralMode.Coast);
		clawMotorLeft.enableVoltageCompensation(true);
		clawMotorLeft.configVoltageCompSaturation(11.0, 0);
		clawMotorLeft.configOpenloopRamp(0.2, 0);
		clawMotorLeft.setInverted(true);
		clawMotorRight.set(ControlMode.PercentOutput, 0);
		clawMotorRight.setInverted(false);
		clawMotorRight.setNeutralMode(NeutralMode.Coast);
		clawMotorRight.enableVoltageCompensation(true);
		clawMotorRight.configVoltageCompSaturation(11.0, 0);
		clawMotorRight.configOpenloopRamp(0.2, 0);

		overrideBumpSwitch(false);
	}

	/**
	 * Adds current protection to the claw motor. If the claw motor trips this, the
	 * claw will stop
	 */
	public void clawMotorsCurrentProtection() {
		clawMotorLeftCurrentTrigger.whenActive(new ClawSetMotorSpeed(0.0));
		clawMotorRightCurrentTrigger.whenActive(new ClawSetMotorSpeed(0.0));
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void openClaw() { // this logic is funky Clean up
		/*
		 * double currentAngle = Robot.armMotor.getArmDegrees(); if (currentAngle <=
		 * RobotMap.angleClawCloseHigh && currentAngle >= RobotMap.angleClawCloseLow) {
		 * } else {
		 */ // no longer need keep-out zone
		clawPiston.set(true); // true is extend
		Robot.log.writeLogEcho("Claw,opening");
		// }
	}

	public void closeClaw() {
		clawPiston.set(false); // false is retract
		Robot.log.writeLogEcho("Claw,closing");
	}

	/**
	 * Checks if claw is open.  Also logs the result to the log file.
	 * @return true = open, false = closed
	 */
	public boolean isClawOpen() {
		return isClawOpen(true);
	}

	/**
	 * Checks if claw is open.
	 * @param log true = also log the result to the log file, false = no logging
	 * @return true = open, false = closed
	 */
	public boolean isClawOpen(boolean log) {
		if (log) Robot.log.writeLog("Claw,isClawOpen," + clawPiston.get());
		return clawPiston.get();
	}

	/**
	 * sets left and right wheel percent power
	 * 
	 * @param percent
	 *            -1 (in) to 1 (out)
	 */
	public void setClawMotorPercent(double percent) {
		clawMotorLeft.set(ControlMode.PercentOutput, percent);
		clawMotorRight.set(ControlMode.PercentOutput, percent);
/**		System.out.println("Left Claw motor " + clawMotorLeft.getDeviceID() + " set to percent " + percent + ", output "
				+ clawMotorLeft.getMotorOutputVoltage() + " V," + clawMotorLeft.getOutputCurrent() + " A, Bus at "
				+ clawMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Claw motor " + clawMotorRight.getDeviceID() + " set to percent " + percent
				+ ", output " + clawMotorRight.getMotorOutputVoltage() + " V," + clawMotorRight.getOutputCurrent()
				+ " A, Bus at " + clawMotorRight.getBusVoltage() + " V");
**/
		Robot.log.writeLogEcho("Claw,set percent," + percent);
		SmartDashboard.putNumber("Left Claw Motor Percent:", percent);
		SmartDashboard.putNumber("Right Claw Motor Percent:", percent);
	}
	
	/**
	 * Returns the speed of the left claw motor
	 * @return precent -1 (in) to +1 (out)
	 */
	public double getClawMotorPercent() {
		return clawMotorLeft.getMotorOutputPercent();
	}

	/**
	 * Reads the value of the bump switch
	 * 
	 * @return true = object is pressing bump switch
	 */
	public boolean getBumpSwitch() {
		if(bumpSwitchOverride) return false;
		return !bumpSwitch.get();
	}

	/**
	 * Reads the value of the photo switch
	 * 
	 * @return true = object is breaking the photo beam
	 */
	public boolean getPhotoSwitch() {
		return photoSwitch.get();
	}

	/**
	 * Turns bump switch override on or off
	 * @param override true = override, false = use sensor
	 */
	public void overrideBumpSwitch(boolean override) {
		bumpSwitchOverride = override;
	}
	
	/**
	 * stops the claw motors sets the claw motors to 0.0
	 */
	public void stop() {
		setClawMotorPercent(0.0);
	}

	/*
	 * public double getClawMotorPower() { double voltage =
	 * (clawMotorLeft.getMotorOutputPercent() +
	 * clawMotorRight.getMotorOutputPercent()) / 2;
	 * SmartDashboard.putNumber("Arm Motor Output Voltage", voltage); return
	 * voltage; }
	 */

	public void periodic() {
		SmartDashboard.putBoolean("PhotoSwitch Triggered", getPhotoSwitch());
		SmartDashboard.putBoolean("Cube Present", getBumpSwitch());
		SmartDashboard.putBoolean("Arm Photo", getPhotoSwitch());

		SmartDashboard.putNumber("Claw Left Motor voltage", clawMotorLeft.getMotorOutputVoltage());
		SmartDashboard.putNumber("Claw Right Motor voltage", clawMotorRight.getMotorOutputVoltage());
		SmartDashboard.putNumber("Claw Left Motor current", clawMotorLeft.getOutputCurrent());
		SmartDashboard.putNumber("Claw Right Motor current", clawMotorRight.getOutputCurrent());

		if (DriverStation.getInstance().isEnabled()) {
			// TODO Is logging ok here?  Log less often?
//			updateClawLog();
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void updateClawLog() {
		Robot.log.writeLog("Claw Motor Left Output Voltage," + clawMotorLeft.getMotorOutputVoltage()
				+ ",Claw Motor Left Output Current," + clawMotorLeft.getOutputCurrent()
				+ ",Claw Motor Left Output Percent," + clawMotorLeft.getMotorOutputPercent()
				+ "Claw Motor Right Output Voltage," + clawMotorRight.getMotorOutputVoltage()
				+ ",Claw Motor Right Output Current," + clawMotorRight.getOutputCurrent()
				+ ",Claw Motor Right Output Percent," + clawMotorRight.getMotorOutputPercent() + ",Claw Open,"
				+ Robot.claw.isClawOpen() + ",Claw PhotoSwitch Triggered," + Robot.claw.getPhotoSwitch()
				+ ",Claw BumpSwitch Triggered," + Robot.claw.getBumpSwitch() + ",Claw BumpSwitch Override," + bumpSwitchOverride);
	}
}
