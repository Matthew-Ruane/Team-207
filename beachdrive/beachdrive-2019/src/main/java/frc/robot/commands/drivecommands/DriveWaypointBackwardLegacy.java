/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.commands.drivecommands;

import frc.robot.Robot;
import frc.robot.constants.ChassisConst;
/*
 * This will drive the robot forwards to a waypoint on the field based on its 
 * original starting position.
 */
import frc.robot.utility.PIDGains;

public class DriveWaypointBackwardLegacy extends DriveWaypointBackward {
//	double leftDistance, rightDistance;
	
    public DriveWaypointBackwardLegacy(double x, double y, double tolerance, double timeout, boolean stopAtEnd, DrivePIDGains driveGains, DrivePIDGains gyroGains) {
        super(x, y, tolerance, timeout, stopAtEnd, driveGains, gyroGains);
    }

    protected boolean isFinished() {
        if ((Robot.chassis.leftDrivePID.onTarget() || Robot.chassis.rightDrivePID.onTarget()) || isTimedOut())
        {
                return true;            
        }
        return false;
    }
}
