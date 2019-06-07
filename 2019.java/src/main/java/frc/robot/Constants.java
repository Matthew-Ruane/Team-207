package frc.robot;

import org.usfirst.frc330.wpilibj.PIDGains;

import frc.utility.*;

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

    public static double Drive_kP = 0.1;
    public static double Drive_kI = 0.0;
    public static double Drive_kD = 0.0;
    public static double Drive_kF = 0.0;
    public static int Drive_kIzone = 0;
    public static int kDriveCruiseVelo = 30000;
    public static int kDriveAccel = 25000;
    public static double kToleranceDistance = 1000;
    public static double TurnOutputMin = 0.2;
    public static double kTurnrateCurve = 0.1;
    public static double kTurnrateLimit = 0.8;
    public static double LeftDistanceTarget, RightDistanceTarget, TurnOutput;

    public static double kToleranceDegrees = 5.0;
    public static double Turn_kP = 0.1;
    public static double Turn_kI = 0.0;
    public static double Turn_kD = 0.35;
    public static double Turn_kF = 0.0;
    public static double DesiredDistance;
    public static double DesiredHeading;

    public static double kTrackLengthInches = 8.265;
    public static double kTrackWidthInches = 23.8;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.5;
    public static double kDriveWheelDiameterInches = 6;

        // PID gains for drive velocity loop (LOW GEAR)
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 0.1; //1.0
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 0.0; //6.0
    public static double kDriveVelocityKf = 0.5;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 4.0; // 6.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50.0;

    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingMaxVel = 120; // inches/sec
    public static double kPathFollowingMaxAccel = 60.0; // inches/sec^2
    
    public static double kLooperDt = 0.01;
    
    public static final boolean On = true;
    public static final boolean Off = false;

    // 12t pinions, 50/24 first reduction, 60/34 second reduction, 20->64 final drive
    // 36/12 encoder
    // 39321.6 ticks per wheel rotation in low gear (experimentally confirmed)


    /* Pathfinder Constants below */
    public static final int encoderTicksPerRevolution = 6542;  // careful when using in math -- this is an int!
    // Imperial versions
    public static final double wheelbase_in = 26.5;       // wheelbase, in inches
    public static final double wheel_diameter_in = 4.0;   // wheel diamater, in inches
    public static final double wheel_distance_in_per_tick = wheel_diameter_in*Math.PI/encoderTicksPerRevolution; //Wheel distance traveled per encoder tick, in inche
    public static final double encoderTicksPerInch = 1950.0; // 347.22*4;
    public static final double max_velocity_ips = 165.0;   // max robot velocity, in inches per second
    public static final double max_acceleration_ipsps = 80.0;  // max robot acceleration, in inches per second per second
    public static final double max_jerk_ipspsps = 2400.0;  // max robot jerk, in inches per second per second per second
    // Metric versions
    public static final double wheelbase_m = wheelbase_in*0.0254;           // wheelbase, in meters
    public static final double wheel_diameter_m = wheel_diameter_in*0.0254; // wheel diamater, in meters
    public static final double wheel_distance_m_per_tick = wheel_distance_in_per_tick*0.0254;  // Wheel distance traveled per encoder tick, in meters
    public static final double max_velocity_mps = max_velocity_ips*0.0254;  // max robot velocity, in meters per second
    public static final double max_acceleration_mpsps = max_acceleration_ipsps*0.0254;  // max robot acceleration, in meters per second per second
    public static final double max_jerk_mpspsps = max_jerk_ipspsps*0.0254;  // max robot jerk, in meters per second per second per second
}
