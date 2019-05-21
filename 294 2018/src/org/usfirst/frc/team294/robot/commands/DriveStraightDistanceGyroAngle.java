package org.usfirst.frc.team294.robot.commands;

import java.util.function.Supplier;

import org.omg.PortableServer.ImplicitActivationPolicyOperations;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.utilities.ToleranceChecker;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Used for when you want to drive straight at some specified angle.
 */
public class DriveStraightDistanceGyroAngle extends Command {

	private double distErr = 0;
	private double distanceTravel, percentPower;
	private ToleranceChecker tolCheck;
	private boolean success;
	private double distSpeedControl;
	
	private final double 
			kPdist = 0.05, 
			kDdist = 0.37,
			kIdist = 0.00; // not used
	
	private double prevDistErr;
	private double angleErr;
	private double intErr = 0;
	private double prevAngleErr;
	
	private double kPangle = .06;
	private double kIangle = .002;
	private double kDangle = .1;
	
	private double curve;
	private double minSpeed = .1;
	private double angleTurn; // in degrees

	public DriveStraightDistanceGyroAngle(double distanceTravel, double percentPower, double angleTurn) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		this.distanceTravel = distanceTravel;
		this.percentPower = percentPower;
		this.angleTurn = angleTurn;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		distErr = 0;
		prevDistErr = 0;
		angleErr = 0;
		success = false;
		distSpeedControl = 0;
		tolCheck = new ToleranceChecker(1, 5);
		Robot.driveTrain.zeroLeftEncoder();
		Robot.driveTrain.zeroRightEncoder();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putNumber("Left Encoder Values", Robot.driveTrain.getLeftEncoderTicks());
		SmartDashboard.putNumber("Right Encoder Values", Robot.driveTrain.getRightEncoderTicks());
		final double currentDistanceInches = Robot.driveTrain.getAverageEncoderInches();
			//	+ Robot.driveTrain.getRightEncoderPosition()) / 2.0);

		distErr = distanceTravel - currentDistanceInches;
		SmartDashboard.putNumber("Distance Error", distErr);
		success = tolCheck.success(Math.abs(distErr));
		if (!success) {
			distSpeedControl = distErr * kPdist + (distErr - prevDistErr) * kDdist;
			SmartDashboard.putNumber("SpeedControl", distSpeedControl);
			prevDistErr = distErr;
			distSpeedControl = distSpeedControl > 1 ? 1 : distSpeedControl;
			distSpeedControl = distSpeedControl < -1 ? -1 : distSpeedControl;
			distSpeedControl *= percentPower;
			if (distSpeedControl > 0) {
				distSpeedControl = (distSpeedControl < minSpeed) ? minSpeed : distSpeedControl;
			} else {
				distSpeedControl = (distSpeedControl > -minSpeed) ? -minSpeed : distSpeedControl;
			}
			angleErr = angleTurn - Robot.driveTrain.getGyroRotation();
			angleErr = (angleErr > 180) ? angleErr - 360 : angleErr;
			intErr = intErr + angleErr * 0.02;
			double dErr = angleErr - prevAngleErr;
			prevAngleErr = angleErr;
			curve = angleErr * kPangle + intErr * kIangle + dErr * kDangle;
			curve = (curve > 0.5) ? 0.5 : curve;
			curve = (curve < -0.5) ? -0.5 : curve;
			curve = (distErr >= 0) ? curve : -curve;
			Robot.driveTrain.driveAtCurve(distSpeedControl, curve);

		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return success;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
