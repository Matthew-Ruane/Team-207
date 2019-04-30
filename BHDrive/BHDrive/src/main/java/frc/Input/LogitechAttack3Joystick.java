package frc.Input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class LogitechAttack3Joystick extends Joystick {

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
    };

    public LogitechAttack3Joystick(int port) {
        super(port);
    }

    public Button getButtonTrigger() {
        return mButtons[1]; 
    } 
    public Button getButton2() { 
        return mButtons[2]; 
    } 
    public Button getButton3() { 
        return mButtons[3]; 
    } 
    public Button getButton4() { 
        return mButtons[4]; 
    } 
    public Button getButton5() { 
        return mButtons[5]; 
    } 
    public Button getButton6() { 
        return mButtons[6]; 
    } 
    public Button getButton7() { 
        return mButtons[7]; 
    } 
    public Button getButton8() { 
        return mButtons[8];
    } 
    public Button getButton9() { 
        return mButtons[9];
    } 
    public Button getButton10() { 
        return mButtons[10]; 
    } 
    public Button getButton11() { 
        return mButtons[11]; 
    } 

    public double getXAxis() { 
        return this.getRawAxis(0); 
    } 
    public double getXAxisInverted() { 
        return -this.getRawAxis(0); 
    } 
    public double getYAxis() { 
        return this.getRawAxis(1); 
    } 
    public double getYAxisInverted() { 
        return -this.getRawAxis(1); 
    } 
    public double getZAxis() { 
        return this.getRawAxis(2); 
    } 
    public double getZAxisInverted() { 
        return 1.0 - this.getRawAxis(2); 
    }
    
}
