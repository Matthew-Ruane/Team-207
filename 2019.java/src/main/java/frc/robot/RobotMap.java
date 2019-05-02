package frc.robot;

public class RobotMap {
    public static final int mDrive_Left_A_ID = 3;
    public static final int mDrive_Left_B_ID = 5;
    public static final int mDrive_Left_C_ID = 7;
    public static final int mDrive_Right_A_ID = 4;
    public static final int mDrive_Right_B_ID = 6;
    public static final int mDrive_Right_C_ID = 8;

    public static final int mShooter_ID = 10;

    public static final int mElevator_Master_ID = 9;
    public static final int mElevator_Slave_ID = 11;

    public static final int mTray_Extend_ID = 1;
    public static final int mTray_Retract_ID = 1;
    public static final int mShift_High_ID = 2;
    public static final int mShift_Low_ID = 2;
    public static final int mTalons_Hold_ID = 3;
    public static final int mTalons_Release_ID = 3;

    public static final int mCargo_Loaded_Sensor_ID = 9;
    public static final int mHatch_Loaded_Sensor_ID = 0;

    public static final int mPCM_A = 2;
    public static final int mPCM_B = 1;

    public static final int LeftStickPort = 0;
    public static final int RightStickPort = 1;
    public static final int GamepadPort = 2;

    public static final int mUltrasonic_Ping_ID = 5;
    public static final int mUltrasonic_Echo_ID = 6;

    public static final boolean On = true;
    public static final boolean Off = false;

    // Robot physical layout
    public static final int encoderTicksPerRevolution = 4096;  // careful when using in math -- this is an int!
    // Imperial versions
    public static final double wheelbase_in = 26.5;       // wheelbase, in inches
    public static final double wheel_diameter_in = 4.0;   // wheel diamater, in inches
    public static final double wheel_distance_in_per_tick = wheel_diameter_in*Math.PI/encoderTicksPerRevolution;  // Wheel distance traveled per encoder tick, in inches
    public static final double max_velocity_ips = 165.0;   // max robot velocity, in inches per second
    public static final double max_acceleration_ipsps = 80.0;  // max robot acceleration, in inches per second per second
    public static final double max_jerk_ipspsps = 2400.0;  // max robot jerk, in inches per second per second per second
    // Metric versions
    public static final double wheelbase_m = wheelbase_in*0.0254;           // wheelbase, in meters
    public static final double wheel_diameter_m = wheel_diameter_in*0.0254; // wheel diamater, in meters
    public static final double wheel_distance_m_per_tick = wheel_diameter_in*0.0254;  // Wheel distance traveled per encoder tick, in meters
    public static final double max_velocity_mps = max_velocity_ips*0.0254;  // max robot velocity, in meters per second
    public static final double max_acceleration_mpsps = max_acceleration_ipsps*0.0254;  // max robot acceleration, in meters per second per second
    public static final double max_jerk_mpspsps = max_jerk_ipspsps*0.0254;  // max robot jerk, in meters per second per second per second
}
