package org.usfirst.frc.team294.robot.commands;

import java.util.function.Supplier;

import org.omg.PortableServer.ImplicitActivationPolicyOperations;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.utilities.ToleranceChecker;
import org.usfirst.frc.team294.utilities.EllipseGenerator;
import org.usfirst.frc.team294.utilities.ProfileGenerator;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Used for when you want to drive straight at some specified angle, uses motion
 * profiling
 */
public class DriveStraightDistanceEllipse extends Command {

	private double distErr = 0;
	// private double distanceTravel;
	private double finalX, finalY;
	private double currentDistance;
	private ToleranceChecker tolCheck;
	private boolean success;
	private double distSpeedControl;

	private final double kPdist = 0.05, kDdist = 0.5, kIdist = 0.00; // not used

	private double prevDistErr;
	private double angleErr;
	private double intErr = 0;
	private double prevAngleErr;
	private double prevDistanceInches;
	private double prevX;
	private double prevY;

	private double kPangle = 0.11;// .06;
	private double kDangle = .2;
	private double kIangle = .002;

	private double curve;
	private double minSpeed = .1;
	private double angleBase;
	// private ProfileGenerator trapezoid;
	private EllipseGenerator ellipse;

	public DriveStraightDistanceEllipse(double x, double y, double angleBase) {
		requires(Robot.driveTrain);
		// this.distanceTravel = distanceTravel;
		this.finalX = x;
		this.finalY = y;
		this.angleBase = angleBase;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// distanceTravel = SmartDashboard.getNumber("DistToTravelDSDG", 60);
		distErr = 0;
		prevDistErr = 0;
		angleErr = 0;
		success = false;
		distSpeedControl = 0;
		tolCheck = new ToleranceChecker(1, 5);
		Robot.driveTrain.zeroLeftEncoder();
		Robot.driveTrain.zeroRightEncoder();
		ellipse = new EllipseGenerator(30, 30, 0, 0, 0);
		// trapezoid = new ProfileGenerator(0.0, distanceTravel, 0, 120, 120);
		prevDistanceInches = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double currentDistanceInches = Robot.driveTrain.getAverageEncoderInches();
		ellipse.updateCurrentPosition();
		this.currentDistance = currentDistanceInches;
		// distErr = trapezoid.getCurrentPosition() - currentDistanceInches;
		distErr = ellipse.getDistErr();
		success = false;// tolCheck.success(Math.abs(distErr));

		if (!success) {
			distSpeedControl = distErr * kPdist + (distErr - prevDistErr) * kDdist;
			distSpeedControl *= 0.5;
			prevDistErr = distErr;
			// distSpeedControl = distSpeedControl > percentPower ? percentPower :
			// distSpeedControl;
			// distSpeedControl = distSpeedControl < -percentPower ? -percentPower :
			// distSpeedControl;
			// TODO change this to max percent power, instead of scaling power
			if (distSpeedControl > 0) {
				distSpeedControl = (distSpeedControl < minSpeed) ? minSpeed : distSpeedControl;
			} else {
				distSpeedControl = (distSpeedControl > -minSpeed) ? -minSpeed : distSpeedControl;
			}
			angleErr = ellipse.getAngleErr();
			angleErr = (angleErr > 180) ? angleErr - 360 : angleErr;
			intErr = intErr + angleErr * 0.02;
			double dAngleErr = angleErr - prevAngleErr;
			prevAngleErr = angleErr;
			curve = angleErr * kPangle + intErr * kIangle + dAngleErr * kDangle;
			curve *= 10;
			// curve *= 0.15;
			// curve = (curve > 0.5) ? 0.5 : curve;
			// curve = (curve < -0.5) ? -0.5 : curve;
			curve = (distErr >= 0) ? curve : -curve;
			//Robot.driveTrain.driveAtCurve(distSpeedControl, curve);

			double diffInches = currentDistanceInches - prevDistanceInches;

			Robot.driveTrain
					.addFieldPositionX(diffInches * Math.cos(Math.toRadians(Robot.driveTrain.getGyroRotation())));
			Robot.driveTrain
					.addFieldPositionY(diffInches * Math.sin(Math.toRadians(Robot.driveTrain.getGyroRotation())));

			prevDistanceInches = currentDistanceInches;
			this.prevX = Robot.driveTrain.getFieldPositionX();
			this.prevY = Robot.driveTrain.getFieldPositionY();
		}

		// SmartDashboard.putNumber("Distance Calculated",
		// trapezoid.getCurrentPosition());
		// SmartDashboard.putNumber("Distance Error", distanceTravel -
		// currentDistanceInches);
		SmartDashboard.putNumber("Actual Distance", currentDistance);
		SmartDashboard.putNumber("Actual Percent Power", distSpeedControl);
		SmartDashboard.putNumber("Actual Curve", curve);
		SmartDashboard.putNumber("Field X", prevX);
		SmartDashboard.putNumber("Field Y", prevY);
		SmartDashboard.putNumber("profile x", ellipse.profileX);
		SmartDashboard.putNumber("Profile Y", ellipse.profileY);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		try {
			return success;
		} finally {
			if (success) {
				// SmartDashboard.putNumber("fDistance Calculated",
				// trapezoid.getCurrentPosition());
				// SmartDashboard.putNumber("fSet Distance", distanceTravel);
				SmartDashboard.putNumber("fActual Distance", currentDistance);
			}
		}
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
