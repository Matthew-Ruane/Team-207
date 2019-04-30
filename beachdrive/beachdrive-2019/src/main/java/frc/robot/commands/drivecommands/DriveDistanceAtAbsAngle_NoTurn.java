// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package frc.robot.commands.drivecommands;
import frc.robot.Robot;
import frc.robot.constants.ChassisConst;
import org.usfirst.frc330.wpilibj.PIDGains;
/**
 *
 */
public class  DriveDistanceAtAbsAngle_NoTurn extends DriveDistance{
    double angle;
    DrivePIDGains gyroGains;
    
    public DriveDistanceAtAbsAngle_NoTurn(double distance, double angle, DrivePIDGains driveGains, DrivePIDGains gyroGains)
    {
        this(distance, 6, angle, -1.0, true, driveGains, gyroGains); //-1 means no timeout
    }
    
    public DriveDistanceAtAbsAngle_NoTurn(double distance, double angle, double tolerance, DrivePIDGains driveGains, DrivePIDGains gyroGains)
    {
        this(distance, tolerance, angle, -1.0, true, driveGains, gyroGains); //-1 means no timeout
    }
    
    public DriveDistanceAtAbsAngle_NoTurn(double distance, double tolerance, double angle, double timeout, boolean stopAtEnd, DrivePIDGains driveGains, DrivePIDGains gyroGains)
    {
        super(distance, tolerance, timeout, stopAtEnd, driveGains);
        this.angle = angle;
        this.gyroGains = gyroGains;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
//    	leftDistance = leftDistance + Robot.chassis.getLeftDistance();
//        rightDistance = rightDistance + Robot.chassis.getRightDistance();
        super.initialize();

        Robot.chassis.gyroPID.setPID(gyroGains);
        Robot.chassis.gyroPID.setSetpoint(angle);
        Robot.chassis.gyroPID.enable();  
    }
}
