/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
/**
 * Add your docs here.
 */
public class LogitechF310 extends Joystick {

    private final Button[] mButtons = {
        new JoystickButton(this, 0),
        new JoystickButton(this, 1), // A Button
        new JoystickButton(this, 2), // B Button
        new JoystickButton(this, 3), // X Button
        new JoystickButton(this, 4), // Y Button
        new JoystickButton(this, 5), // Left Bumper
        new JoystickButton(this, 6), // Right Bumper
        new JoystickButton(this, 7), // Back Button
        new JoystickButton(this, 8), // Start Button
        new JoystickButton(this, 9), // Left Stick Button
        new JoystickButton(this, 10), // Right Stick Button
        new JoystickButton(this, 11),
        new JoystickButton(this, 12)
    };

    public LogitechF310(int port) {
        super(port);
    }
    //Axes
    public double getLeftX() {
        return getRawAxis(1);
    }
    public double getLeftY() {
        return getRawAxis(2);
    }
    public double getRightX() {
        return getRawAxis(4);
    }
    public double getRightY() {
        return getRawAxis(5);
    }
    //Buttons
    public Button getButtonA() {
        return mButtons[0];
    }
    public Button getButtonB() {
        return mButtons[3];
    }
    public Button getButtonX() {
        return mButtons[1];
    }
    public Button getButtonY() {
        return mButtons[4];
    }
    public Button getLeftBumper() {
        return mButtons[5];
    }
    public Button getRightBumper() {
        return mButtons[6];
    }
    public Button getLeftTrigger() {
        return mButtons[7];
    }
    public Button getRightTrigger() {
        return mButtons[8];
    }
    public Button getButtonBack() {
        return mButtons[9];
    }
    public Button getButtonStart() {
        return mButtons[10];
    }
    public Button getButtonLeftStick() {
        return mButtons[11];
    }
    public Button getButtonRightStick() {
        return mButtons[12];
    }
}
