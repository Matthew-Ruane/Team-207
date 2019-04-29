package frc.robot;

public class Constants {
    
    public static final double kEncoderTicksPerInch = 574;
    public static double kPosition = 0;
    public static final double COLLECT_CARGO = 1.0;
    public static final double ROCKET_BOTTOM_HEIGHT_CARGO = 5.3;
    public static final double ROCKET_MID_HEIGHT_CARGO = 33.5;
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
}
