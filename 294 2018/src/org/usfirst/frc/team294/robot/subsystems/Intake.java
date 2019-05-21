
package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
//import org.usfirst.frc.team294.robot.commands.IntakeMotorSetToZero;
import org.usfirst.frc.team294.robot.triggers.MotorCurrentTrigger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
/**
public class Intake extends Subsystem {

	private final Solenoid intakeOpenPiston = new Solenoid(RobotMap.pneumaticIntakePistonOpen);
	private final DoubleSolenoid intakeDeployPiston = new DoubleSolenoid(RobotMap.pneumaticIntakePistonDeploy,
			RobotMap.pneumaticIntakePistonStow); // Forward = deployed

	private final TalonSRX intakeMotorLeft = new TalonSRX(RobotMap.intakeMotorLeft);
	private final TalonSRX intakeMotorRight = new TalonSRX(RobotMap.intakeMotorRight);
	private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchIntake);

	private static boolean cubeInIntake;
	private double lastMotorCurrent = 0;
	private double motorCurrent = 0;

	private DoubleSolenoid.Value intakeState = DoubleSolenoid.Value.kOff;

	public final MotorCurrentTrigger intakeMotorLeftCurrentTrigger = new MotorCurrentTrigger(intakeMotorLeft, 8, 4);
	public final MotorCurrentTrigger intakeMotorRightCurrentTrigger = new MotorCurrentTrigger(intakeMotorRight, 8, 4);

	public Intake() {
		intakeMotorLeft.set(ControlMode.PercentOutput, 0);
		intakeMotorLeft.setNeutralMode(NeutralMode.Brake);
		intakeMotorLeft.enableVoltageCompensation(true);
		intakeMotorLeft.configVoltageCompSaturation(11.0, 0);
		intakeMotorLeft.setInverted(true);

		intakeMotorRight.set(ControlMode.PercentOutput, 0);
		intakeMotorRight.setNeutralMode(NeutralMode.Brake);
		intakeMotorRight.enableVoltageCompensation(true);
		intakeMotorRight.configVoltageCompSaturation(11.0, 0);
		intakeMotorRight.setInverted(false);

		cubeInIntake = false;
	}

	@Override
	protected void initDefaultCommand() {}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	/**
	 * Adds current protection to the intake motor. If the intake motor trips this,
	 * the intake will stop
	 *
	public void intakeMotorsCurrentProtection() {
		intakeMotorLeftCurrentTrigger.whenActive(new IntakeMotorSetToZero());
		intakeMotorRightCurrentTrigger.whenActive(new IntakeMotorSetToZero());
	}

	/**
	 * Returns the state of the intake grabbers.
	 * 
	 * @return true = open, false = closed
	 *
	public boolean isIntakeOpen() {
		Robot.log.writeLog("Intake,isOpen?," + intakeOpenPiston.get());
		return intakeOpenPiston.get();
	}

	/**
	 * Deploys or retracts the intake based on parameter
	 * 
	 * @param deployed
	 *            true = deployed, false = retracted
	 *
	public void setIntakeDeploy(boolean deployed) {
		// Track intake state in software (variable intakeState), so that we
		// can read back the intake state change faster than the CanBus
		// roundtrip
		if (!deployed) {
			if (Robot.armMotor.getArmDegrees() > (RobotMap.armIntakeClearanceAng + 3)) {
				intakeState = DoubleSolenoid.Value.kReverse;
				Robot.log.writeLogEcho("Intake,Retracting,arm is high");
			} else if (Robot.armMotor.getArmDegrees() < (RobotMap.minAngle + 6)) {   // was minAngle+3
				intakeState = DoubleSolenoid.Value.kReverse;
				Robot.log.writeLogEcho("Intake,Retracting,arm is low");
			} else {
				intakeState = DoubleSolenoid.Value.kForward;
				Robot.log.writeLogEcho("Intake,Deploying (even though commanded to retract),arm is in keep-out region");
			}
		} else {
			intakeState = DoubleSolenoid.Value.kForward;
			Robot.log.writeLogEcho("Intake,Deploying");
		}
		intakeDeployPiston.set(intakeState);
		// stop();
	}

	/**
	 * Open or close the jaws on the intake
	 * 
	 * @param open
	 *            true = open, false = close
	 *
	public void setIntakeOpen(boolean open) {
		Robot.log.writeLogEcho("Intake,SetOpen," + (open ? "open" : "closed"));
		intakeOpenPiston.set(open);
	}

	// TODO:  This doesn't work properly.  Delete me
	public void updateCubeStatus() {
		cubeInIntake = !cubeInIntake;
	}

	// TODO:  This doesn't work properly.  Delete me
	/**
	 * Doesn't work, don't use?
	 * @return
	 *
	public boolean isCubeInIntake() {
		return cubeInIntake;
	}

	/**
	 * Returns true if intake motor is decreasing by more than 1 amp from last call to this method.
	 * @return
	 *
	public boolean isCurrentDecreasing() {
		motorCurrent = intakeMotorLeft.getOutputCurrent();
		Robot.log.writeLog("Intake,isCurrentDecreasing?,current," + motorCurrent + ",decreasing," + ((motorCurrent-lastMotorCurrent)<-1));
//		SmartDashboard.putNumber("Intake Left Motor Delta Current", motorCurrent - lastMotorCurrent);
		if ((motorCurrent - lastMotorCurrent) < -1) {
			lastMotorCurrent = 0;
			return true;
		} else {
			lastMotorCurrent = motorCurrent;
			return false;
		}
	}

	// public void setIntakeMotorToPercentPower(double leftPercent, double
	// rightPercent) {
	// intakeMotorLeft.set(ControlMode.PercentOutput, leftPercent);
	// intakeMotorRight.set(ControlMode.PercentOutput, rightPercent);
	// System.out.println("Left Intake motor " + intakeMotorLeft.getDeviceID() + "
	// set to percent " + leftPercent
	// + ", output " + intakeMotorLeft.getMotorOutputVoltage() + " V," +
	// intakeMotorLeft.getOutputCurrent()
	// + " A, Bus at " + intakeMotorLeft.getBusVoltage() + " V");
	// System.out.println("Right Intake motor " + intakeMotorRight.getDeviceID() + "
	// set to percent " + rightPercent
	// + ", output " + intakeMotorRight.getMotorOutputVoltage() + " V," +
	// intakeMotorRight.getOutputCurrent()
	// + " A, Bus at " + intakeMotorRight.getBusVoltage() + " V");
	// SmartDashboard.putNumber("Left Intake Motor Percent:", leftPercent);
	// SmartDashboard.putNumber("Right Intake Motor Percent:", rightPercent);
	// }

	/**
	 * sets the intake motors to a percentage
	 * 
	 * @param percent
	 *            -1 (out) to 1 (in)
	 *
	public void setIntakeMotorPercent(double percent) {
		intakeMotorLeft.set(ControlMode.PercentOutput, percent);
		intakeMotorRight.set(ControlMode.PercentOutput, percent);
/**		System.out.println("Left Intake motor " + intakeMotorLeft.getDeviceID() + " set to percent " + percent
				+ ", output " + intakeMotorLeft.getMotorOutputVoltage() + " V," + intakeMotorLeft.getOutputCurrent()
				+ " A, Bus at " + intakeMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Intake motor " + intakeMotorRight.getDeviceID() + " set to percent " + percent
				+ ", output " + intakeMotorRight.getMotorOutputVoltage() + " V," + intakeMotorRight.getOutputCurrent()
				+ " A, Bus at " + intakeMotorRight.getBusVoltage() + " V");
**		SmartDashboard.putNumber("Left Intake Motor Percent:", percent);
		SmartDashboard.putNumber("Right Intake Motor Percent:", percent);
		
		Robot.log.writeLogEcho("Intake,setMotorPercent," + percent);
	}

	/**
	 * Sets the intake motors to opposite directions
	 * @param percentPower -1.0 (right out, left in) to +1.0 (right in, left out)
	 *
	public void setIntakeMotorPercentOpposite(double percentPower) {
		intakeMotorLeft.set(ControlMode.PercentOutput, -percentPower);
		intakeMotorRight.set(ControlMode.PercentOutput, percentPower);

		Robot.log.writeLogEcho("Intake,Opposite setMotorPercent," + percentPower);
	}

	/**
	 * Reads the Talon speed of the left intake motor
	 * @return -1 to +1, percent power
	 *
	public double getIntakeMotorPercent() {
		return intakeMotorLeft.getMotorOutputPercent();
	}
	
	/**
	 * closes the intake jaws if the photo switch is triggered
	 * 
	 * @return true once claw is closed, false otherwise
	 *
	public boolean smartCloseIntake() {
		// if object is detected with photoSwitch, close the intake
		if (photoSwitch.get()) {
			setIntakeOpen(false);
			return true;
		} else
			return false;
	}

	/**
	 * stops the intake motors
	 *
	public void stop() {
		setIntakeMotorPercent(0.0);
	}

	/**
	 * Reads the value of the photo switch
	 * 
	 * @return true = object is breaking the photo beam
	 *
	public boolean getPhotoSwitch() {
		return photoSwitch.get();
	}

	/**
	 * Gets the state of the intake
	 * 
	 * @return true = deployed, false = retracted or unknown state
	 *
	public boolean isIntakeDeployed() {
		Robot.log.writeLog("Intake,isDeployed?," + (intakeState == DoubleSolenoid.Value.kForward));
		return intakeState == DoubleSolenoid.Value.kForward;
	}

	public void periodic() {
		SmartDashboard.putBoolean("Object Present (Intake): ", getPhotoSwitch());
		SmartDashboard.putBoolean("Intake Photo", photoSwitch.get());
		SmartDashboard.putNumber("Intake Left Motor voltage", intakeMotorLeft.getMotorOutputVoltage());
		SmartDashboard.putNumber("Intake Right Motor voltage", intakeMotorRight.getMotorOutputVoltage());
		SmartDashboard.putNumber("Intake Left Motor current", intakeMotorLeft.getOutputCurrent());
		SmartDashboard.putNumber("Intake Right Motor current", intakeMotorRight.getOutputCurrent());
		SmartDashboard.putBoolean("Intake deployed", intakeDeployPiston.get() == DoubleSolenoid.Value.kForward);

		if (DriverStation.getInstance().isEnabled()) {
			// TODO Is logging ok here?  Log less often?
//			updateIntakeLog();
		}
	}

	public void updateIntakeLog() {
		Robot.log.writeLog("Intake Motor Left Output Voltage," + intakeMotorLeft.getMotorOutputVoltage()
				+ ",Intake Motor Left Output Current," + intakeMotorLeft.getOutputCurrent()
				+ ",Intake Motor Left Output Percent," + intakeMotorLeft.getMotorOutputPercent()
				+ ",Intake Motor Right Output Voltage," + intakeMotorRight.getMotorOutputVoltage()
				+ ",Intake Motor Right Output Current," + intakeMotorRight.getOutputCurrent()
				+ ",Intake Motor Right Output Percent," + intakeMotorRight.getMotorOutputPercent() + ",Intake Deployed,"
				+ Robot.intake.isIntakeDeployed() + ",Intake Open," + Robot.intake.isIntakeOpen()
				+ ",Intake PhotoSwitchTriggered," + Robot.intake.isCubeInIntake());
	}
}
**/
