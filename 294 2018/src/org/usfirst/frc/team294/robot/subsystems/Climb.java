package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Compressor;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team294.robot.commands.*;
import edu.wpi.first.wpilibj.DriverStation;

/**
 *
 */
public class Climb extends Subsystem {

	// We are using the power plug for climbMotor1 to run the intake LEDs, so
	// commenting out the talon until we get power to it.
	private final TalonSRX climbMotor1 = new TalonSRX(RobotMap.climbMotor1);
	private final TalonSRX climbMotor2 = new TalonSRX(RobotMap.climbMotor2);
	private final Solenoid climbPiston = new Solenoid(RobotMap.pneumaticClimbPistonRetract);
	private final Compressor compressor = new Compressor(0);

	public Climb() {
		// Configure talons
		climbMotor2.set(ControlMode.PercentOutput, 0);
		climbMotor2.setNeutralMode(NeutralMode.Brake);
		climbMotor2.enableVoltageCompensation(true);
		climbMotor2.configVoltageCompSaturation(11.0, 0);
		climbMotor1.set(ControlMode.PercentOutput, 0);
		climbMotor1.setNeutralMode(NeutralMode.Brake);
		climbMotor1.enableVoltageCompensation(true);
		climbMotor1.configVoltageCompSaturation(11.0, 0);
		climbMotor1.set(ControlMode.Follower, RobotMap.climbMotor2);
		
		enableCompressor(true);
	}

	/**
	 * Enables or disables the compressor.
	 * @param turnOn true = turn on compressor when pressure drops.
	 *    false = keep compressor off (to prevent brownouts).
	 */
	public void enableCompressor(boolean turnOn) {
		compressor.setClosedLoopControl(turnOn);
	}
	
	/**
	 * Deploys Climb Piston, but only in the last 30 seconds of the match
	 * @param deploy true = deploy, false = retract
	 */
	public void deployClimbPiston(boolean deploy) {
		//Commented out for practice because there is no match time
		//if (DriverStation.getInstance().getMatchTime() <= 30) {
		climbPiston.set(deploy);
		Robot.claw.stop();		// stop claw motors
		Robot.log.writeLogEcho("Climb hook,deploy," + deploy);
//		}
	}

	/**
	 * Sets the climb motor based on a percent power parameter
	 * 
	 * @param percentPower -1 (climb) to +1 (fall)
	 */
	public void setClimbMotors(double percentPower) {
		// If running climb motor, turn off compressor to reduce brownout likelihood
		compressor.setClosedLoopControl(percentPower==0);		

		climbMotor2.set(ControlMode.PercentOutput, percentPower);
		Robot.log.writeLogEcho("Climb motor,percent power," + percentPower);
	}
	
	/**
	 * Returns the climb motor power setting
	 * @return percentPower, -1 (climb) to +1 (fall)
	 */
	public double getClimbMotorPercentPower() {
		return climbMotor2.getMotorOutputPercent();
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	public void periodic() {
		if(DriverStation.getInstance().isEnabled() && (Math.abs(getClimbMotorPercentPower()) > 0.1) ) {
			updateClimbLog();
		}
	}
	
	public void updateClimbLog() {
		Robot.log.writeLog("Climb voltage 1," + climbMotor1.getMotorOutputVoltage()
				+ ",Climb percent 1," + climbMotor1.getMotorOutputPercent()
				+ ",Climb current 1," + climbMotor1.getOutputCurrent()
				+ ",Climb voltage 2," + climbMotor2.getMotorOutputVoltage()
				+ ",Climb percent 2," + climbMotor2.getMotorOutputPercent()
				+ ",Climb current 2," + climbMotor2.getOutputCurrent());
	}
}
