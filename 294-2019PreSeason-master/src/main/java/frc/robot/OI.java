/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  // USB device IDs
  public Joystick leftJoystick = new Joystick(0);
	public Joystick rightJoystick = new Joystick(1);
	public Joystick coPanel = new Joystick(2);
  public Joystick xboxController = new Joystick(3);
  

  public OI() {

    Button[] left = new Button[12];
    Button[] right = new Button[12];
    Button[] coP = new Button[15];
    Button[] xbB = new Button[10];
    
    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);

      left[i].whenPressed(new Shift(false));
      right[i].whenPressed(new Shift(true));
    }

    // Declare codriver panel switches
    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // Xbox controller buttons
    for (int i = 1; i < xbB.length; i++) {
      xbB[i] = new JoystickButton(xboxController, i);
    }

    // Smartdashboard buttons to test commands
    SmartDashboard.putData("FileLog Update Time", new FileLogUpdateDateTime());
    SmartDashboard.putData("Pathfinder Test", new PathfinderTest1());
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


}
