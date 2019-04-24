/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Input.*;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private static final LogitechAttack3Joystick LeftStick = new LogitechAttack3Joystick(RobotMap.LeftStickPort);
    private static final LogitechAttack3Joystick RightStick = new LogitechAttack3Joystick(RobotMap.RightStickPort);
    private static final LogitechF310 Gamepad = new LogitechF310(RobotMap.GamepadPort);

    public void registerControls() {
        //Driver methods to rapidly transition to collecting state for either game piece
        LeftStick.getButton5().whenPressed(new CollectCargoCommand());
        RightStick.getButton4().whenPressed(new CollectHatchCommand());
        //manual override of talon control
        LeftStick.getButton2().whenPressed(new TalonsReleaseCommand());
        LeftStick.getButton3().whenPressed(new TalonsHoldCommand());
        //cargo shooting control***May not stop shooting cargo when released, if not create stopshootcargo and implement on getButtonTrigger().whenReleased
        RightStick.getButtonTrigger().whileHeld(new ShootCargoCommand());
        //gamepad methods to move elevator to different positions
        Gamepad.getButtonA().whenPressed(new RocketBottomCommand());
        Gamepad.getButtonB().whenPressed(new RocketMidCommand());
        Gamepad.getButtonY().whenPressed(new RocketTopCommand());
        //Fallback Gamepad method to switch between ball and hatch mode
        Gamepad.getButtonBack().whenPressed(new CargoModeCommand());
        Gamepad.getButtonStart().whenPressed(new HatchModeCommand());
        }

    public static double getThrottleInput() {
        return RightStick.getYAxis();
    }
    public static double getSteeringInput() {
        return LeftStick.getXAxis();
    }
    public static double getThrottleInputInverted() {
        return RightStick.getYAxisInverted();
    }
    public static double getSteeringInputInverted() {
        return LeftStick.getXAxisInverted();
    }
    


}

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
