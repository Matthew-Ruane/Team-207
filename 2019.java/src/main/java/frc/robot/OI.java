package frc.robot;

import frc.robot.Input.*;
import frc.robot.commands.*;

public class OI {

    private static final LogitechAttack3Joystick LeftStick = new LogitechAttack3Joystick(RobotMap.LeftStickPort);
    private static final LogitechAttack3Joystick RightStick = new LogitechAttack3Joystick(RobotMap.RightStickPort);
    private static final LogitechF310 Gamepad = new LogitechF310(RobotMap.GamepadPort);

    public void registerControls() {
        //Driver methods to rapidly transition to collecting state for either game piece
        RightStick.getButton5().whenPressed(new CollectCargoCommand());
        RightStick.getButton4().whenPressed(new CollectHatchCommand());
        //manual override of talon control
        LeftStick.getButton2().whenPressed(new TalonsReleaseCommand());
        LeftStick.getButton3().whenPressed(new TalonsHoldCommand());
        //cargo shooting control
        RightStick.getButtonTrigger().whenPressed(new ShootCargoCommand());
        //gamepad methods to move elevator to different positions
        Gamepad.getButtonA().whenPressed(new RocketBottomCommand());
        Gamepad.getButtonB().whenPressed(new RocketMidCommand());
        Gamepad.getButtonY().whenPressed(new RocketTopCommand());
        Gamepad.getButtonX().whenPressed(new CargoshipCommand());
        //Fallback Gamepad method to switch between ball and hatch mode
        Gamepad.getButtonBack().whenPressed(new CargoModeCommand());
        Gamepad.getButtonStart().whenPressed(new HatchModeCommand());
        //Toggling extension of tray with left trigger
        LeftStick.getButtonTrigger().whenPressed(new TrayExtensionToggle());
        //zeroing Elevator encoder and NavX yaw gyro
        LeftStick.getButton9().whenPressed(new ZeroGyro());
        LeftStick.getButton7().whenPressed(new PathfinderTest1());
        
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
