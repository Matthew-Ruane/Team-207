package frc.robot;


import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Input.LogitechAttack3Joystick;
import frc.Input.LogitechF310;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	private static OI instance;

	private LogitechF310 m_driver;
	

	private OI() {
		m_driver = new LogitechF310(2);
	}
	
	public static OI getInstance() {
		if(instance == null) {
			instance = new OI();
		}
		return instance;
	}
	public LogitechF310 getDriverController() {
		return m_driver;
	}

}

