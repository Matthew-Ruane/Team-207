package frc.robot;

public class Constants {
    
    public static final double kEncoderTicksPerInch = 574;
    public static double kPosition = 0;
    public static final double COLLECT_CARGO = 1.0;
    public static final double ROCKET_BOTTOM_HEIGHT_CARGO = 3.3;
    public static final double ROCKET_MID_HEIGHT_CARGO = 31.5;
    public static final double ROCKET_TOP_HEIGHT_CARGO = 57.5;
    public static final double SHIP_HEIGHT_CARGO = 19.5;
    public static final double SHIP_HEIGHT_HATCH = 3.3;
    public static final double COLLECT_HATCH = 4.3;
    public static final double ROCKET_BOTTOM_HEIGHT_HATCH = 4.3;
    public static final double ROCKET_MID_HEIGHT_HATCH = 32.5;
    public static final double ROCKET_TOP_HEIGHT_HATCH = 58.5;

    public static final int CARGO_STATE_LOADED = 0;
    public static final int CARGO_STATE_UNLOADED = 1;
    public static int CARGO_STATE = CARGO_STATE_UNLOADED;

    public static final int TRAY_STATE_EXTENDED = 0;
    public static final int TRAY_STATE_RETRACTED = 1;
    public static int TRAY_STATE = TRAY_STATE_RETRACTED;

    public static final int TALON_STATE_HOLDING = 0;
    public static final int TALON_STATE_RELEASED = 1;
    public static int TALON_STATE = TALON_STATE_HOLDING;

    public static final int HIGH_GEAR = 0;
    public static final int LOW_GEAR = 1;
    public static int CURRENT_GEAR = HIGH_GEAR;

    public static boolean WantHatch = false;

    public static final int ElevatorSlotIDx = 0;
    public static final double Elevator_kP = 0.40;
    public static final double Elevator_kI = 0.0;
    public static final double Elevator_kD = 30.0;
    public static final int Elevator_MotionAccel = 13000;
    public static final int Elevator_MotionCruiseVelo = 15000;
    public static final int kTimeoutms = 10;

    public static double kDrive_Motion_P = 1.4;				// %/ft
	public static double kDrive_Motion_D = 0.0;				// %/(ft/s)
	public static double kDrive_Motion_V = 0.058;			// %/(ft/s) max turn speed
	public static double kDrive_Motion_A = 0.0;	

    public static double kDrive_Motion_turnP = 0.75;
    public static double kDrive_Motion_turnI = 0.5;
    public static double kDrive_Motion_turnD = 0.2;

    public static final double kGearRatio = 4.8;
    public static final double kEncoderDriveRatio = 5.4;
    public static double kDrive_Motion_trackwidth = 2.72;
	public static double kDrive_WheelDiameterInch = 3.875;
    
    public static double getWheelCircumference() { 
        return (kDrive_WheelDiameterInch*Math.PI)/12.0; 
    }
    public static double getTicksPerWheelRotation() {
        return (kGearRatio*4096);
    }
    public static double getDistancePerTick() {
        return (getWheelCircumference()/getTicksPerWheelRotation());
    }
}
