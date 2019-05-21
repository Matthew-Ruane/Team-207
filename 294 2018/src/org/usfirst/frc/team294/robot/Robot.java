
package org.usfirst.frc.team294.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.subsystems.*;
import org.usfirst.frc.team294.utilities.AutoSelection;
import org.usfirst.frc.team294.utilities.FileLog;
import org.usfirst.frc.team294.utilities.LEDSet;
import org.usfirst.frc.team294.utilities.RobotPreferences;
import org.usfirst.frc.team294.utilities.VisionData;

public class Robot extends TimedRobot {

	// Subsystem objects
	public static DriveTrain driveTrain;
	public static Shifter shifter;
	public static ArmPiston armPiston;
	public static ArmMotor armMotor;
	public static Claw claw;
//	public static Intake intake;
	public static OI oi;
	public static Climb climb;
	public static PressureSensor pressureSensor;	

	public static LEDSet mainLEDs;
	
	public static FileLog log;
	public static RobotPreferences robotPrefs;

	public static VisionData visionData;
	public static AutoSelection autoSelection;
	public static UsbCamera driveCamera;

	public static boolean prototypeRobot; // Set true if using code for prototype, false for practice and competition
	public static boolean driveDirection; // true for reversed
	
	public static boolean beforeFirstEnable = true;  // true before the first time the robot is enabled after booting or loading code

	public NetworkTableInstance networkTables;
	public NetworkTable coDisplay;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Create the log file first, so that any other code can use the file log
		log = new FileLog(); 

		// Read robot preferences **before** creating subsystems, so subsytems can use
		// the preferences
		robotPrefs = new RobotPreferences();
		
		// Set variable that the robot has not been enabled
		beforeFirstEnable = true;

		// Create Compressor, Vision and LED objects before subsystems
		
		visionData = new VisionData();
		mainLEDs = new LEDSet(RobotMap.LEDMain);
		
		// Create subsystems
		driveTrain = new DriveTrain();
		shifter = new Shifter();
		armPiston = new ArmPiston();
		armMotor = new ArmMotor();
		claw = new Claw();
		climb = new Climb();
//		intake = new Intake();
		pressureSensor = new PressureSensor();

		// armMotor.armMotorsCurrentProtection(); needs to be tested
//		intake.intakeMotorsCurrentProtection();
		claw.clawMotorsCurrentProtection();
		
		// Reset single-sided solenoid to default state, so solenoid doesn't move when we download new code
		climb.deployClimbPiston(false);	
		climb.enableCompressor(true);

		// Create auto selection utility
		autoSelection = new AutoSelection();

		// Network Tables for driver's display
		networkTables = NetworkTableInstance.getDefault();
		coDisplay = networkTables.getTable("coDisplay"); // I think this will work, just need to send value to it

		/**
		 * Comment out UsbCamera if the video info is sent through RaspberryPi
		 **/
		// USB drive camera
//		driveCamera = CameraServer.getInstance().startAutomaticCapture();
		
		// There seems to be some issues with the RoboRio camera driver.  Don't use this code until the driver is fixed?
//		driveCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 160, 120, 15); 
//		driveCamera.setExposureAuto(); // Start in auto exposure mode so that we can set brightness 
//		driveCamera.setBrightness(30); // Setting brightness only works correctly in auto exposure mode (?)  was 10
//		driveCamera.getProperty("contrast").set(80);
//		driveCamera.getProperty("saturation").set(60); 
//		driveCamera.setExposureManual(20);
//		driveCamera.setWhiteBalanceManual(2800);

		// Create the OI last, so that it can use commands that call subsystems
		oi = new OI();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		log.writeLogEcho("Robot disabled.");
		climb.setClimbMotors(0);
		armMotor.joystickControl = false;
		climb.deployClimbPiston(false);
		climb.enableCompressor(true);
		oi.setXBoxRumble(0);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		log.writeLogEcho("Autonomous mode started.");
		
		// Set variable that the robot has been enabled
		beforeFirstEnable = false;
		
		// Read and set overrides, as applicable
		OverrideSensor override = new OverrideSensor();   // Use booleans from SmartDashboard
		override.setOverrides();   						// Set Overrides
		
		autoSelection.readGameData();

		driveTrain.zeroLeftEncoder();
		driveTrain.zeroRightEncoder();
		driveTrain.zeroGyroRotation();

		climb.enableCompressor(true);
		
		// schedule the autonomous command
		if (autoSelection.autonomousCommand != null) {
			Command shiftLow = new Shift(false);
			shiftLow.start();
			claw.closeClaw();
//			new IntakeSetDeploy(false);
			autoSelection.autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.getGyroRotation();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autoSelection.autonomousCommand != null) {
			autoSelection.autonomousCommand.cancel();
		}

		driveTrain.zeroGyroRotation(); // TODO remove later
		driveTrain.setFieldPositionX(0); // TODO remove later
		driveTrain.setFieldPositionY(0); // TODO remove later

		climb.enableCompressor(true);
		
		log.writeLogEcho("Teleop mode started.");
		
		// Set variable that the robot has been enabled
		beforeFirstEnable = false;
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		if (claw.isClawOpen(false) && claw.getClawMotorPercent()<-0.4) {
			oi.setXBoxRumble(0.7);
		} else {
			oi.setXBoxRumble(0);
		}

/*		
		// Control LED colors
		if (climb.getClimbMotorPercentPower() < -0.1) {
			// Robot is climbing
			Robot.oi.setXBoxRumble(0);
			Robot.mainLEDs.setRed();
		} else if (climb.getClimbMotorPercentPower() > 0.1) {
			Robot.oi.setXBoxRumble(0);
			Robot.mainLEDs.setBlue();
		} else if (intake.getIntakeMotorPercent() > 0.1) {
			Robot.oi.setXBoxRumble(0.7);
			Robot.mainLEDs.setPurple();
		} else {
			Robot.oi.setXBoxRumble(0);
			Robot.mainLEDs.setOff();
		}
		*/
	}


	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
