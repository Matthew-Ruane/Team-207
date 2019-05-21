package org.usfirst.frc.team294.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem controlling the two pistons on the arm
 */
public class ArmPiston extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private final DoubleSolenoid armPistonMajor = new DoubleSolenoid(RobotMap.pneumaticArmPistonMajorOut, RobotMap.pneumaticArmPistonMajorIn);
	private final DoubleSolenoid armPistonMajor2 = new DoubleSolenoid(RobotMap.pneumaticArmPistonMajorOut2, RobotMap.pneumaticArmPistonMajorIn2);
//	private final Solenoid armPistonMinorSingle = new Solenoid(RobotMap.pneumaticArmPistonMinorOut);

	// Limit switches for pistons
	private final DigitalInput majorLimitRetract = new DigitalInput(RobotMap.majorPistonRetractedLimitSwitch);
//	private final DigitalInput majorLimitExtend = new DigitalInput(RobotMap.majorPistonExtendedLimitSwitch);
//	private final DigitalInput minorLimitRetract = new DigitalInput(RobotMap.minorPistonRetractedLimitSwitch);
//	private final DigitalInput minorLimitExtend = new DigitalInput(RobotMap.minorPistonExtendedLimitSwitch);

	// Tracking piston position in software
	private PistonPositions majorPosition = PistonPositions.Null;
	private PistonPositions minorPosition = PistonPositions.Null;
	private boolean overrideArm;
	
	public ArmPiston() {
		super();
		overrideArmSensor(false);
	}

	private boolean getMajorRet() {
		if(overrideArm) return true;
		return !majorLimitRetract.get();
	}

	/**
	 * Turns Arm piston retract override on or off
	 * @param override true = override, false = use sensor
	 */
	public void overrideArmSensor(boolean override) {
		overrideArm = override;
	}
	
	/**
	 * 
	 * @return true if arm piston sensor is being overridden
	 */
	public boolean getOverride() {
		return overrideArm;
	}
/*	private boolean getMajorExt() {
		return !majorLimitExtend.get();
	}

	private boolean getMinorRet() {
		return !minorLimitRetract.get();
	}

	private boolean getMinorExt() {
		return !minorLimitExtend.get();
	} *\

	/**
	 * Sets the position of the major piston
	 * 
	 * @param position
	 *            only accepts PistonPositions.Extended and
	 *            PistonPositions.Retracted </br>
	 *            <b>Other values are ignored</b>
	 */
	public void setMajor(RobotMap.PistonPositions position) {
		if (position == RobotMap.PistonPositions.Extended) {
			armPistonMajor.set(Value.kForward);
			armPistonMajor2.set(Value.kForward);
			Robot.log.writeLogEcho("Arm Piston,Extend");
		}

		if (position == RobotMap.PistonPositions.Retracted) {
			armPistonMajor.set(Value.kReverse);
			armPistonMajor2.set(Value.kReverse);
			Robot.log.writeLogEcho("Arm Piston,Retract");
		}
	}

	
	/**
	 * Sets the position of the minor piston
	 * 
	 * @param position
	 *            only accepts PistonPositions.Extended and
	 *            PistonPositions.Retracted </br>
	 *            <b>Other values are ignored</b>
	 */
/*	public void setMinor(RobotMap.PistonPositions position) {
			if (position == RobotMap.PistonPositions.Extended)
				armPistonMinorSingle.set(true);
			if (position == RobotMap.PistonPositions.Retracted)
				armPistonMinorSingle.set(false);
	}
*/
	/**
	 * returns position of major piston according to sensors
	 * 
	 * @return
	 */
	public PistonPositions getMajor() {
		switch (majorPosition) {
//		case Extended:
//			SmartDashboard.putString("Major Piston", "Extended");
//			break;
		case Retracted:
			SmartDashboard.putString("Major Piston", "Retracted");
			break;
		case Moving:
			SmartDashboard.putString("Major Piston", "Moving");
			break;
		case Null:
			SmartDashboard.putString("Major Piston", "Null");
			break;
		}
		return majorPosition;
	}

	/**
	 * returns position of minor piston according to sensors
	 * 
	 * @return
	 */
/*	public PistonPositions getMinor() {
		switch (minorPosition) {
		case Extended:
			SmartDashboard.putString("Minor Piston", "Extended");
			break;
		case Retracted:
			SmartDashboard.putString("Minor Piston", "Retracted");
			break;
		case Moving:
			SmartDashboard.putString("Minor Piston", "Moving");
			break;
		case Null:
			SmartDashboard.putString("Minor Piston", "Null");
			break;
		}
		return minorPosition; *\
	}

	/**
	 * Updates the SmartDashboard. Called every 20ms by the piston subsystem.
	 */
	public void updateSmartDashboard() {
//		SmartDashboard.putBoolean("Major Extend", getMajorExt());
		SmartDashboard.putBoolean("Major Retract", getMajorRet());
//		SmartDashboard.putBoolean("Minor Extend", getMinorExt());
//		SmartDashboard.putBoolean("Minor Retract", getMinorRet());
		getMajor();
//		getMinor();
	}

	/**
	 * Updates the states of the piston. See RobotMap.PistonPositions for
	 * values.</br>
	 * Called every 20ms by the piston subsystem.
	 */
	public void updateState() {
//		if (getMajorExt())
//			majorPosition = PistonPositions.Extended;
		if (getMajorRet())
			majorPosition = PistonPositions.Retracted;
		else
			majorPosition = PistonPositions.Moving;
	}

/*		if (getMinorExt())
			minorPosition = PistonPositions.Extended;
		else if (getMinorRet())
			minorPosition = PistonPositions.Retracted;
		else
			minorPosition = PistonPositions.Moving; *\

		// Should we add a check to see if both limit switches are activated, and if so,
		// set the position to null?
		// That would indicate faulty hardware, and we wouldn't want to move the arm
		// into danger zones with faulty hardware
	}

	/**
	 * Extends both pistons only if in the proper zone
	 * 
	 * @return IS NOT CORRECT true if pistons changed state, false if otherwise
	 */
	public boolean smartExtend() {
		//  TODO fix return values
		ArmZones zone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		if (zone == ArmZones.Low) {
			setMajor(PistonPositions.Extended);	// Allow arm to extend in Low zone.  Was "Retracted"
			//setMinor(PistonPositions.Extended);
//			setMinor(PistonPositions.Retracted);
			return true;
		} else if (zone == ArmZones.High) {
			setMajor(PistonPositions.Extended);
//			setMinor(PistonPositions.Extended);
			return true;
		} else {
			setMajor(PistonPositions.Retracted);
//			setMinor(PistonPositions.Retracted);
			return false;
		}
	}

	/**
	 * Extends the major piston <b>only if</b> legal to do so (i.e. in high zone)
	 * 
	 * @return true if piston extended, false otherwise
	 */
	public boolean smartExtendMajor() {
		ArmZones zone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		if (zone == ArmZones.High || zone == ArmZones.Low) {
			setMajor(PistonPositions.Extended);
			return true;
		} else {
			setMajor(PistonPositions.Retracted);
			return false;
		}
	}

	/**
	 * Extends the minor piston <b>only if</b> legal to do so (i.e. in high or low
	 * zone)
	 * 
	 * @return true if piston extended, false otherwise
	 */
/*	public boolean smartExtendMinor() {
		ArmZones zone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		if (zone == ArmZones.High || zone == ArmZones.Low) {
			setMinor(PistonPositions.Extended);
			return true;
		} else {
			setMinor(PistonPositions.Retracted);
			return false;
		}
	}
*/
	/**
	 * Retracts both pistons
	 */
	public void smartRetract() {
		setMajor(PistonPositions.Retracted);
//		setMinor(PistonPositions.Retracted);
	}

	public void periodic() {
		updateState();
		updateSmartDashboard();
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}