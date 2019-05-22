package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * Author Matt Ruane (207)
 */
public class Tray extends Subsystem {

    private static final Tray instance = new Tray();

    public static Tray getInstance() {
      return instance;
    }
    
    public static Solenoid mTray_Extend, mTray_Retract, mTalons_Hold, mTalons_Release;
    public static TalonSRX mShooter;
    public static DigitalInput mCargo_Loaded_Sensor, mHatch_Loaded_Sensor;

    public static boolean autograb;

    public Tray() {
        mTray_Extend = new Solenoid(RobotMap.mPCM_B, RobotMap.mTray_Extend_ID);
        mTray_Retract = new Solenoid(RobotMap.mPCM_A, RobotMap.mTray_Retract_ID);
        mTalons_Hold = new Solenoid(RobotMap.mPCM_A, RobotMap.mTalons_Hold_ID);
        mTalons_Release = new Solenoid(RobotMap.mPCM_B, RobotMap.mTalons_Release_ID);
        mShooter = new TalonSRX(RobotMap.mShooter_ID);

        mShooter.setNeutralMode(NeutralMode.Brake);
        mShooter.configContinuousCurrentLimit(30);
        mShooter.configPeakCurrentLimit(0);
        mShooter.enableCurrentLimit(true);
        mShooter.setInverted(false);

        mCargo_Loaded_Sensor = new DigitalInput(RobotMap.mCargo_Loaded_Sensor_ID);
        mHatch_Loaded_Sensor = new DigitalInput(RobotMap.mHatch_Loaded_Sensor_ID);

        autograb = false;
    }

    public static void ShootCargo() {
        mShooter.set(ControlMode.PercentOutput, 1.0);
    }
    public static void StopShootCargo() {
        mShooter.set(ControlMode.PercentOutput, 0.0);
    }
    public static void IntakeCargo() {
        mShooter.set(ControlMode.PercentOutput, 1.0);
    }
    public static void StopIntakeCargo() {
        mShooter.set(ControlMode.PercentOutput, 0.0);
    }
    public static void ExtendTray() {
        mTray_Extend.set(Constants.On);
        mTray_Retract.set(Constants.Off);
        Constants.TRAY_STATE = Constants.TRAY_STATE_EXTENDED;
    }
    public static void RetractTray() {
        mTray_Extend.set(Constants.Off);
        mTray_Retract.set(Constants.On);
        Constants.TRAY_STATE = Constants.TRAY_STATE_RETRACTED;
    }
    public static void TalonsHold() {
        mTalons_Release.set(Constants.Off);
        mTalons_Hold.set(Constants.On);
        Constants.TALON_STATE = Constants.TALON_STATE_HOLDING;
    }
    public static void TalonsRelease() {
        mTalons_Hold.set(Constants.Off);
        mTalons_Release.set(Constants.On);
        Constants.TALON_STATE = Constants.TALON_STATE_RELEASED;
    }
    public static void TalonsAutoGrab() {
            if (Constants.WantHatch == true && !mHatch_Loaded_Sensor.get()) {
                autograb = false;
                TalonsHold();
                Constants.WantHatch = false;
            }
            else if (Constants.WantHatch == true && mHatch_Loaded_Sensor.get()){
                autograb = true;
                TalonsRelease();
            }
    }
    public static void UpdateLoadState() {
        if (!mCargo_Loaded_Sensor.get()) {
            Constants.CARGO_STATE = Constants.CARGO_STATE_LOADED;
        }
        else if (mCargo_Loaded_Sensor.get()) {
            Constants.CARGO_STATE = Constants.CARGO_STATE_UNLOADED;
        }
    }
    
    @Override
    public void initDefaultCommand() {
    }
}
