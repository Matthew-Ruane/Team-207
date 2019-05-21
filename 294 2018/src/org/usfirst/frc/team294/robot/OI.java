  
package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.OI.BottomKnob;
import org.usfirst.frc.team294.robot.OI.MiddleKnob;
import org.usfirst.frc.team294.robot.OI.TopKnob;
import org.usfirst.frc.team294.robot.RobotMap.ArmPositions;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.commands.OverrideSensor.Sensors;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team294.robot.commands.autoroutines.*;
import org.usfirst.frc.team294.robot.subsystems.PressureSensor;
import org.usfirst.frc.team294.robot.triggers.AxisTrigger;
import org.usfirst.frc.team294.robot.triggers.POVTrigger;
import org.usfirst.frc.team294.utilities.AutoSelection;
import org.usfirst.frc.team294.utilities.AutoSelection.AutoPlan;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units;

public class OI {

	// Threshold position checker
	// For top and bottom knobs unless otherwise specified
	double[] knobThreshold = new double[] { -0.911, -0.731, -0.551, -0.367, -0.1835, -0.0035, 0.1775, 0.3605, 0.5455,
			0.7285, 0.91 };
	// For middle knob only
	double[] middleKnobThreshold = new double[] { -0.751, -0.25, 0.2525, 0.7525 };

	public enum TopKnob {
		minus6degrees, minus5degrees, minus4degrees, minus2degrees, minus3degrees, minus1degree, noChange, plus1degree, plus2degrees, plus3degrees, plus4degrees, plus5degrees
	}

	public enum MiddleKnob {
		PositionOne, PositionTwo, PositionThree, PositionFour, PositionFive
	}

	public enum BottomKnob {
		PositionOne, PositionTwo, PositionThree, PositionFour, PositionFive, PositionSix, PositionSeven, PositionEight, PositionNine, PositionTen, PositionEleven, PositionTwelve
	}

	TopKnob[] TopKnobPositions = new TopKnob[] { TopKnob.minus6degrees, TopKnob.minus5degrees, TopKnob.minus4degrees,
			TopKnob.minus3degrees, TopKnob.minus2degrees, TopKnob.minus1degree, TopKnob.noChange, TopKnob.plus1degree,
			TopKnob.plus2degrees, TopKnob.plus3degrees, TopKnob.plus4degrees, TopKnob.plus5degrees };
	MiddleKnob[] MiddleKnobPositions = new MiddleKnob[] { MiddleKnob.PositionOne, MiddleKnob.PositionTwo,
			MiddleKnob.PositionThree, MiddleKnob.PositionFour, MiddleKnob.PositionFive };
	BottomKnob[] BottomKnobPositions = new BottomKnob[] { BottomKnob.PositionOne, BottomKnob.PositionTwo,
			BottomKnob.PositionThree, BottomKnob.PositionFour, BottomKnob.PositionFive, BottomKnob.PositionSix,
			BottomKnob.PositionSeven, BottomKnob.PositionEight, BottomKnob.PositionNine, BottomKnob.PositionTen,
			BottomKnob.PositionEleven, BottomKnob.PositionTwelve };

	// Optional commands based on start position
	Command[] MiddleKnobCommands = new Command[] {
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand()
	};

	// Commands based on alliance abilities (auto routines)
	Command[] BottomKnobCommands = new Command[] {
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
			// new AutoCommand(),
	};

	public Joystick leftJoystick = new Joystick(0); // Left Joystick is in port 0
	public Joystick rightJoystick = new Joystick(1); // Right Joystick is in port 1
	public Joystick coPanel = new Joystick(2);
	public Joystick xboxController = new Joystick(3);
	// public Joystick armJoystick = new Joystick(4); // Arm Joystick is in port 4

	private boolean driveDirection = true; // Easy switching drive direction

	SendableChooser<AutoPlan> chooser_autoPlan = new SendableChooser<>();
	SendableChooser<StartingPosition> chooser_startPosition = new SendableChooser<>();

	public OI() {

		// Create button arrays for the input devices
		Button[] left = new Button[12];
		Button[] right = new Button[12];
		Button[] coP = new Button[15];
		Button[] xbB = new Button[11];
		Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
		Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);

		Trigger xbPovUp = new POVTrigger(xboxController, 0);
		Trigger xbPovRight = new POVTrigger(xboxController, 90);
		Trigger xbPovDown = new POVTrigger(xboxController, 180);
		Trigger xbPovLeft = new POVTrigger(xboxController, 270);

		// Declare left and right joystick buttons
		for (int i = 1; i < left.length; i++) {
			left[i] = new JoystickButton(leftJoystick, i);
			right[i] = new JoystickButton(rightJoystick, i);
			if (i == 1) {
				left[i].whenPressed(new CubeLetGo()); // Automatic cube pick up with vision
				right[i].whenPressed(new CubeShootOut()); // Automatic cube pick up with vision
			} else if (i == 3) {
				//right[i].whenPressed(new ClimbMotorSequence()); // Switch drive
				//left[i].whenPressed(new ClimbPreparation());
				 // direction
				// left[i].whenPressed(new SwitchDriveDirection(false)); // Switch drive
				// direction
			} else if (i == 2) {

				// right[i].whenPressed(); // Auto driving routines
				// left[i].whenPressed(); // Auto driving routines

				right[i].whenPressed(new Shift(false));
				left[i].whenPressed(new Shift(true));

			} else if (i == 4) {
				right[i].whenPressed(new DriveWithJoysticks());
				left[i].whenPressed(new DriveWithJoysticks());
			}else if(i ==5) {
				 right[i].whenPressed(new CubeShootOut(1));
			}
		}

		// Declare codriver panel switches
		for (int i = 1; i < coP.length; i++) {
			coP[i] = new JoystickButton(coPanel, i);
		}

		// Xbox controller buttons
		for (int i = 1; i < xbB.length; i++) {
			xbB[i] = new JoystickButton(xboxController, i);
		}

		// Bind commands to the codriver panel switches
		coP[1].whileHeld(new ClimbSetPercentPower(RobotMap.climbPercent, false)); // Preps for climb
		coP[2].whenPressed(new ClimbMotorSequence()); // Does  climb sequence
		coP[3].whileHeld(new ClimbSetPercentPower(RobotMap.climbLowerRobot,false)); // Lower robot from climbing
		coP[4].whenPressed(new LoadCubeManual()); // 
		coP[5].whenPressed(new ClimbSetPercentPower(RobotMap.climbHoldRobot)); // Hold robot up after climbing
		coP[6].whenPressed(new OverrideSensor(Sensors.Bump)); // Intake mechanism up
		coP[7].whenPressed(new OverrideSensor(Sensors.ArmPiston)); // Intake mechanism down
		coP[8].whenPressed(new ClimbPreparation()); // Arm to intake position
//		coP[9].whenPressed(new IntakeSetSpeed(RobotMap.intakePercentOut)); // Outtake
		coP[10].whenPressed(new ArmMoveWithPiston(ArmPositions.ScaleHigh)); // Arm to backwards for scale
		coP[11].whenPressed(new LoadCubeSequence()); // Intake sequence
		// coP[12].whenPressed(new Command()); // TBD
		coP[13].whenPressed(new ArmMoveWithPiston(ArmPositions.ScaleLow)); // Arm to switch position
		coP[14].whenPressed(new ArmMoveWithPiston(ArmPositions.Switch)); // Arm to alternate scale position

		// Xbox controller buttons

		// xbB[1].whenPressed(new Command()); // Lower intake mechanism
		// xbB[2].whenPressed(new PistonCommand()); // Arm Piston actuation
		// xbB[3].whenPressed(new PistonCommand()); // Arm Piston actuation
		// xbB[4].whenPressed(new ClawSetState(true)); // Open Claw
		// xbB[5].whenPressed(new ArmPistonsRetract()); // Retract Pistons
		// xbB[6].whenPressed(new LoadCubeSequence()); // Intake Sequence
		// xbB[7].whenPressed(new ClimbCommand()); // Reserved for climbing
		// xbB[8].whenPressed(new ClimbCommand()); // Reserved for climbing
		// xbB[9].whenPressed(new OverrideCommand()); // Override climb OR arm

		xbB[1].whenPressed(new LoadCubeSequence()); // grabs cube fully
		xbB[2].whenPressed(new StopIntakeAndClawAndClimb()); // Stops all flywheels
		xbB[3].whenPressed(new LoadCubeSequence(ArmPositions.PortalIntake, PistonPositions.Retracted, false)); 
//		xbB[4].whenPressed(new ToggleIntakeDeploy()); 
		xbB[5].whenPressed(new ArmMoveWithPiston(ArmPositions.ScaleHigh)); 
		xbB[6].whenPressed(new ArmMoveWithPiston(ArmPositions.ScaleLow)); 
		xbB[7].whenPressed(new ToggleClawOpen());
		// xbB[8].whenPressed(new ClimbCommand()); // Reserved for climbing
//		xbB[8].whenPressed(new ToggleIntakeOpen());
		// xbB[9].whenPressed(new ToggleClawOpen());
		// xbB[9].whenPressed(new OverrideCommand()); // Override climb OR arm
		xbB[9].toggleWhenPressed(new ArmMotorControlJoystick()); // Manual Arm Control
		 xbB[10].whenPressed(new ArmMoveWithPiston(37, RobotMap.PistonPositions.Null));

		xbPovUp.whenActive(new ArmMoveWithPiston(ArmPositions.Intake));
		xbPovDown.whenActive(new ArmMoveWithPiston(ArmPositions.StartPosition)); // Arm to intake position
		xbPovLeft.whenActive(new ArmPistonRetract());// Arm to switch position
		xbPovRight.whenActive(new ArmPistonSmartExtend()); // Arm to scale fowards
		// Xbox triggers
		xbLT.whenActive(new ArmMoveWithPiston(ArmPositions.Switch)); 
		xbRT.whenActive(new CubeShootOut(1)); // Score cube
//		xbRT.whenActive(new IntakeShootOut()); // Score cube

		// Button armButton2 = new JoystickButton(armJoystick, 2);
		// Button armButton3 = new JoystickButton(armJoystick, 3);
		// armButton2.whenPressed(new ArmMotorIncrementAngle(false));
		// armButton3.whenPressed(new ArmMotorIncrementAngle(true));

		// Initialize our auto plan chooser
		// Software is okay for testing. This should be hardware switches at
		// competition.
		chooser_autoPlan.addObject("do Closest, if both far do scale", AutoPlan.ClosestSwitchScale_FFScale);
		chooser_autoPlan.addObject("do Closest, if both far do switch from front",
				AutoPlan.ClosestSwitchScale_FFSwitchFront); // Should never need this routine
		chooser_autoPlan.addObject("do closest, if both far do switch from back",
				AutoPlan.ClosestSwitchScale_FFSwitchBack);
		chooser_autoPlan.addObject("do Scale only", AutoPlan.ScaleOnly);
		chooser_autoPlan.addDefault("do Switch only from middle", AutoPlan.SwitchOnly);
		chooser_autoPlan.addObject("Go to baseline", AutoPlan.BaselineOnly);
		chooser_autoPlan.addObject("Go to baseline", AutoPlan.BaselineOnly);		
		chooser_autoPlan.addObject("Dead Reckoning Baseline Forwards", AutoPlan.DeadReckoningForward);
		chooser_autoPlan.addObject("Same side switch or baseline", AutoPlan.SameSideSwitchOrBaseline);
		// chooser_autoPlan.addObject("2Cube", 5);
		SmartDashboard.putData("Auto Plan Selection", chooser_autoPlan);
		SmartDashboard.putData("AutoTest1", new AutoTest1());
		SmartDashboard.putData("Auto 5 Middle Auto Switch", new AutoPath5_SwitchFromMiddle(true));
		chooser_startPosition.addObject("Left", StartingPosition.Left);
		chooser_startPosition.addDefault("Middle", StartingPosition.Middle);
		chooser_startPosition.addObject("Right", StartingPosition.Right);

		// Smart Dashboard Commands
		SmartDashboard.putData("Start Position Selection", chooser_startPosition);

		SmartDashboard.putData("Start Drive Train", new DriveWithJoysticks());

		SmartDashboard.putData("Retract Arm", new ArmPistonSetMajorState(PistonPositions.Retracted));

		SmartDashboard.putData("Extend Arm", new ArmPistonSetMajorState(PistonPositions.Extended));

		// SmartDashboard.putData("Control Arm Motor Joystick", new ArmMotorControl());
		SmartDashboard.putData("Calibrate arm zero position", new ArmMotorCalibrateZero());

		SmartDashboard.putData("Move Arm to Legal Area", new ArmMoveToLegalRange());
		SmartDashboard.putData("Move to Edge of Range", new ArmMoveToEdge(90));

		SmartDashboard.putData("Score Backwards in Scale", new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, RobotMap.PistonPositions.Null));
		SmartDashboard.putData("Move arm to 0 and set piston state", new ArmMoveWithPiston(0.0, RobotMap.PistonPositions.Null));
		SmartDashboard.putData("Score Forwards in Scale", new ArmMoveWithPiston(RobotMap.armScaleLowPos, RobotMap.PistonPositions.Null));
		SmartDashboard.putData("Score Forwards in Scale (extend)",
				new ArmMoveWithPiston(RobotMap.armScaleLowPos, RobotMap.PistonPositions.Extended));
		SmartDashboard.putData("Score in Switch Low", new ArmMoveWithPiston(RobotMap.armSwitchPosLow, RobotMap.PistonPositions.Null));
		SmartDashboard.putData("Intake Position", new ArmMoveWithPiston(RobotMap.armIntakePos, RobotMap.PistonPositions.Null));

		SmartDashboard.putData("Arm Intake Cube", new ArmIntakeCube());
//		SmartDashboard.putData("Intake Retract", new IntakeSetDeploy(false));
//		SmartDashboard.putData("Intake Deploy", new IntakeSetDeploy(true));

		// SmartDashboard.putData("Arm Piston Retract Based on Arm Position", new
		// ArmControl());
		SmartDashboard.putData("Control Arm Motor Joystick", new ArmMotorControlJoystick());

		SmartDashboard.putData("Set Arm Position", new SetArmFromSmartDashboard());

		SmartDashboard.putNumber("set Arm Angle Piston Extend and Retract", 90);

		SmartDashboard.putNumber("set Arm Angle", 0);

		SmartDashboard.putData("Move Piston Within Parameters", new ArmMoveToLegalRange());

		SmartDashboard.putData("Turn heckla small", new TurnGyro(90, Units.Degrees));
		SmartDashboard.putData("Auto Switch Shoot", new AutoSwitchShoot());

		SmartDashboard.putData("Retract Arm Pistons", new ArmPistonRetract());

		SmartDashboard.putData("DriveStraightDistanceProfile", new DriveStraightDistanceProfile(30, 0));

		// Drive straight (DriveStraightDistanceProfile) using dashboard distance and
		// parameters
		SmartDashboard.putNumber("DSDP_Distance_inches", 30);
		SmartDashboard.putNumber("DSDP_AngleBase", 0);
		SmartDashboard.putNumber("DSDP_Speed_ips", 80);
		SmartDashboard.putNumber("DSDP_Accel_ips2", 80);
		SmartDashboard.putData("DSDP_Go!", new DriveStraightDistanceProfile());
		
		SmartDashboard.putData("EllipseTest", new DriveStraightDistanceEllipse(30, 10, 0));
		SmartDashboard.putNumber("DistToTravelDSDG", 150);
		SmartDashboard.putData(" ProfileTest", new DriveStraightDistanceEllipse(100, 1000, 0));

		SmartDashboard.putData("Turn with vision", new TurnGyro());
		SmartDashboard.putData("Turn +5", new TurnGyro(5, Units.Degrees));
		SmartDashboard.putData("Turn +10", new TurnGyro(10, Units.Degrees));
		SmartDashboard.putData("Turn +20", new TurnGyro(20, Units.Degrees));
		SmartDashboard.putData("Turn -5", new TurnGyro(-5, Units.Degrees));
		SmartDashboard.putData("Turn -10", new TurnGyro(-10, Units.Degrees));
		SmartDashboard.putData("Turn -20", new TurnGyro(-20, Units.Degrees));
		SmartDashboard.putData("Turn -90", new TurnGyro(-90, Units.Degrees));
		SmartDashboard.putData("Turn +90", new TurnGyro( 90, Units.Degrees));

		SmartDashboard.putData("Release Cube", new CubeLetGo());
		SmartDashboard.putData("Shoot Out Cube", new CubeShootOut());

		SmartDashboard.putData("Extend", new ArmPistonSmartExtendInDestZone(90));

		SmartDashboard.putData("Open Claw", new ClawSetOpen(true));
		SmartDashboard.putData("Close Claw", new ClawSetOpen(false));

		SmartDashboard.putData("Set Climb Motor to 50% forwards", new ClimbSetPercentPower(.50));
		SmartDashboard.putData("Set Climb Motor to 50% backwards", new ClimbSetPercentPower(-.50));
		SmartDashboard.putData("Stop Climb Motor", new ClimbSetPercentPower(0));

		SmartDashboard.putData("Climb Preperation", new ClimbPreparation());
		SmartDashboard.putData("Climb Sequence", new ClimbMotorSequence());
		
		SmartDashboard.putBoolean("Override Arm Retract Sensor", false);
		SmartDashboard.putBoolean("Override Bump Switch", false);
		SmartDashboard.putData("Override Sensors", new OverrideSensor());
		

//		SmartDashboard.putData("Intake Cube", new LoadCubeSequenceWithIntake());
//		SmartDashboard.putData("Open Intake", new IntakeSetOpen(true));
//		SmartDashboard.putData("Close Intake", new IntakeSetOpen(false));

//		SmartDashboard.putData("Invert Intake", new IntakeMotorsSetOpposite());
//		SmartDashboard.putData("Re-Intake Cube", new IntakeInvertAndGrab());

//		SmartDashboard.putData("Intake Sequence with Arm Move", new LoadCubeSequenceWithIntake());

		SmartDashboard.putBoolean("Arm Intake Interlocked", false);
		
//		SmartDashboard.putData("Auto Spin and Intake", new AutoRotateAndIntakeDiamondCube());

		// Camera settings are crashing the Rio kernel camera driver, so don't use
//    	SmartDashboard.putNumber("Camera Exposure (-1 auto)" , 30);
//    	SmartDashboard.putNumber("Camera Brightness" , 30);
//    	SmartDashboard.putData("Camera Set Bright Expose", new CameraSetProperties());
	}

	/**
	 * Sets the Xbox controller rumble power.
	 * 
	 * @param percentRumble,
	 *            value 0 to 1
	 */
	public void setXBoxRumble(double percentRumble) {
		xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
		xboxController.setRumble(RumbleType.kRightRumble, percentRumble);
	}

	/**
	 * Reads the top knob.
	 * 
	 * @return Raw position 0 (full ccw) to 11 (cw)
	 */
	public TopKnob readTopKnob() {
		double knobReading;
		int i = 0;

		knobReading = coPanel.getRawAxis(4);
		int len = knobThreshold.length;
		for (i = 0; i < len; i++) {
			if (knobReading < knobThreshold[i])
				break;
		}

		if (i == len)
			return TopKnobPositions[len - 1];
		return TopKnobPositions[i];
	}

	/**
	 * Reads the middle knob.
	 * 
	 * @return Raw position 0 (full ccw) to 4 (cw). Positions above 4 are
	 *         indeterminate due to resistors missing.
	 */
	public int readMiddleKnobRaw() {
		double knobReading2;

		int i = 0;
		knobReading2 = coPanel.getRawAxis(6);
		int len = middleKnobThreshold.length;
		for (i = 0; i < len; i++) {
			if (knobReading2 < middleKnobThreshold[i])
				break;
		}

		return i;
	}

	/**
	 * Reads the middle knob.
	 * 
	 * @return OI.MiddleKnob robot starting position constant for the current knob
	 *         position
	 */
	public MiddleKnob readMiddleKnob() {
		return MiddleKnobPositions[readMiddleKnobRaw()];
	}

	/**
	 * Gets "drive, turn, and shoot" command based on the robot starting position,
	 * as per the middle knob setting.
	 * 
	 * @return Command to turn and shoot
	 */
	public Command getMiddleKnobCommand() {
		int i;

		i = readMiddleKnobRaw();
		if (i < MiddleKnobCommands.length) {
			return MiddleKnobCommands[i];
		} else {
			return null;
		}
	}

	/**
	 * Reads the bottom knob.
	 * 
	 * @return Raw position 0 (full ccw) to 11 (full cw)
	 */
	public int readBottomKnobRaw() {
		double knobReading;
		int i = 0;

		knobReading = coPanel.getRawAxis(3);
		int len = knobThreshold.length;
		for (i = 0; i < len; i++) {
			if (knobReading < knobThreshold[i])
				break;
		}

		if (i == len)
			return (len - 1);
		return (i);
	}

	/**
	 * Reads the bottom knob.
	 * 
	 * @return OI.BottomKnob barrier constant for the current knob position
	 */
	public BottomKnob readBottomKnob() {
		return BottomKnobPositions[readBottomKnobRaw()];
	}

	/**
	 * Gets autonomous command to run based on bottom knob setting
	 * 
	 * @return Command to run
	 */
	public Command getAutonomousCommand() {
		return BottomKnobCommands[readBottomKnobRaw()];
	}

	public AutoPlan readAutoPlan() {
		return chooser_autoPlan.getSelected();
	}

	public StartingPosition readStartPosition() {
		return chooser_startPosition.getSelected();
	}

	/**
	 * Sets the drive direction
	 * 
	 * @param direction
	 *            false = back in the front, true = intake in the front
	 */
	public void setDriveDirection(boolean direction) {
		this.driveDirection = direction;
	}

	/**
	 * Gets the drive direction of the robot
	 * 
	 * @return false = back in the front, true = intake in the front
	 */
	public boolean getDriveDirection() {
		return driveDirection;
	}
}