package frc.Input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class LogitechF310 extends Joystick {

    private final Button[] mButtons = {
        new JoystickButton(this, 0),
        new JoystickButton(this, 1),
        new JoystickButton(this, 2),
        new JoystickButton(this, 3),
        new JoystickButton(this, 4),
        new JoystickButton(this, 5),
        new JoystickButton(this, 6),
        new JoystickButton(this, 7),
        new JoystickButton(this, 8),
        new JoystickButton(this, 9),
        new JoystickButton(this, 10),
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
        return mButtons[1];
    }
    public Button getButtonB() {
        return mButtons[2];
    }
    public Button getButtonX() {
        return mButtons[3];
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
    public double getLeftTrigger() {
        return getRawAxis(2);
    }
    public Double getRightTrigger() {
        return getRawAxis(3);
    }
    public Button getButtonBack() {
        return mButtons[7];
    }
    public Button getButtonStart() {
        return mButtons[8];
    }
    public Button getButtonLeftStick() {
        return mButtons[9];
    }
    public Button getButtonRightStick() {
        return mButtons[10];
    }
}
