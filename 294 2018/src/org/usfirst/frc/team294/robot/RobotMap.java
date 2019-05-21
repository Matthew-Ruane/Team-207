
package org.usfirst.frc.team294.robot;

public class RobotMap {

	// Hardware CAN addresses
	public static final int leftMotor1 = 10;
	public static final int leftMotor2 = 11; // Master
	public static final int leftMotor3 = 12;
	public static final double encoderTicksPerRevolution = 4096.0;

	public static final int rightMotor1 = 20;
	public static final int rightMotor2 = 21; // Master
	public static final int rightMotor3 = 22;
	public static final int armMotor1 = 30; // Master
	public static final int armMotor2 = 31;
	public static final int clawMotorLeft = 40;
	public static final int clawMotorRight = 41;
//	public static final int intakeMotorLeft = 51;
//	public static final int intakeMotorRight = 50;
	public static final int climbMotor1 = 60;
	public static final int climbMotor2 = 61;

	// Pneumatic addresses
	public static final int pnuematicShifter = 0;
	public static final int pneumaticArmPistonMajorIn2 = 1;
//	public static final int pneumaticIntakePistonOpen = 2;
	public static final int pneumaticClawPistonOut = 3;
	public static final int pneumaticArmPistonMajorIn = 4;
	public static final int pneumaticArmPistonMajorOut = 5;
	public static final int pneumaticClimbPistonRetract = 6;
	public static final int pneumaticArmPistonMajorOut2 = 7;

	// RoboRIO digital I/O addresses
	public static int photoSwitchIntake = 2;
	public static int photoSwitchClaw = 3;
	public static int bumpSwitchClaw = 4;
	public static final int majorPistonRetractedLimitSwitch = 5;

	// RoboRIO analog I/O addresses
	public static final int pressureSensor = 0;

	// RoboRio relay channels
	public static final int LEDMain = 0;

	// intake motor speeds
	public static double intakePercentIn = 0.7; // need to be tested
	public static double intakePercentOut = -0.4; // Slow speed for outtake
	public static double intakePercentShootOut = -0.66; // Fast speed for shooting cube out
	public static double intakePercentIntakeShootOut = -1.0; // TODO speed needs to be checked in command

	// claw motor speeds
	public static double clawPercentDefault = -0.35; // Constant intake speed to prevent cube from dropping out of claw
	public static double clawPercentIn = -1; // need to be tested
	public static double clawPercentInFully = -1.0; // Used to make sure cube is in claw TODO check if necessary
	public static double clawPercentOut = 0.3; // Slow speed for outtake
	public static double clawPercentLetGo = 0.5; // Speed to help drop cube
	public static double clawPercentShootOut = 0.66; // Fast speed for shooting cube out
	public static double clawPercentSwitchShoot = 0.75; // Beginning outtake speed of claw for scoring in switch
	
	// climb motor speeds
	public static double climbPercent = -1.0;
	public static double climbHoldRobot = -0.3;
	public static double climbLowerRobot = 0.2;

	// drive train variables for motion profile
	public static double maxSpeed = 120;  // was 180
	public static double maxAcceleration = 100;  // was 120
	
	// Arm angle thresholds
//	public static double lowThreshold; // Low threshold for ground pickup  Not Used?
//	public static double highThreshold; // High threshold for scoring

	// Arm Scoring Angles - NEED TO BE UPDATED WITH TESTING
	public static double armIntakePos = -39.0;   // Was -53
	public static double armSwitchPosHigh = 5.0;
	public static double armSwitchPosLow = -5.0;
	public static double armScaleLowPos = 60.0;
	public static double armScaleBackwardsPos = 110.0;
	public static double armScaleBackwardsPosAuto = 113.0;
	public static double armClimbPos = 87.0;
	public static double armStartPosition = -53;
	// Arm interlocking angle
	public static double armIntakeClearanceAng = -18.0;
	public static double armPortalIntakePos = -15;
	// Arm angle constants
	public static double degreesPerTicks = 360.0 / 4096.0;

	public enum ArmPositions {
		Intake(armIntakePos), Switch(armSwitchPosHigh), ScaleLow(armScaleLowPos), ScaleHigh(armScaleBackwardsPos), StartPosition(armStartPosition), PortalIntake(armPortalIntakePos);

		private double angle;

		ArmPositions(double angle) {
			this.angle = angle;
		}

		public double getAngle() {
			return angle;
		}
	}

	/**
	 * Gets the angle preset for the arm based on RobotMap.ArmPositions
	 * 
	 * @param position
	 *            RobotMap.ArmPositions
	 * @return
	 */
	public static double getArmAngle(ArmPositions position) {
		if (position == ArmPositions.Intake)
			return armIntakePos;
		if (position == ArmPositions.ScaleHigh)
			return armScaleBackwardsPos;
		if (position == ArmPositions.ScaleLow)
			return armScaleLowPos;
		if (position == ArmPositions.Switch)
			return armSwitchPosHigh;
		else
			return armSwitchPosHigh;
	}

	// Arm angle zone boundaries THESE ARE NUMBERS JUST FOR TEST ON PROTO ARM
	public static double minAngle = -53; // arm cannot extend downward past this angle
	public static double groundBound = -42;  // long piston cannot extend past this angle because it would hit the ground
	public static double lowerBound = -28; // piston1 can be extended between Ang0 and Ang1, cube picked up below Ang1
	public static double middleBound = 35; // arm cannot extend between Ang1 and Ang2
	public static double upperBound = 113; // both pistons can be extended between Ang2 and Ang 3
	public static double maxAngle = 130; // arm cannot extend upward past this angle

	public enum ArmZones {
		Ground, Low, Middle, High, Backwards
	}

	// arm angles where claw pistons cannot be opened
	public static double angleClawCloseLow = -20;
	public static double angleClawCloseHigh = -10;

	/**
	 * Returns the zone of the arm based on the arm angle
	 * 
	 * @param angle
	 * @return RobotMap.ArmZones
	 */
	public static ArmZones getArmZone(double angle) {
		if (angle <= groundBound)
			return ArmZones.Ground;
		if (angle < lowerBound)
			return ArmZones.Low;
		if (angle >= upperBound)
			return ArmZones.Backwards;
		if (angle > middleBound)
			return ArmZones.High;
		return ArmZones.Middle;
	}

	public enum PistonPositions {
		Extended, Retracted, Moving, Null
	}
}
