/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

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
    
    private static final Solenoid mTray_Extend = new Solenoid(RobotMap.mPCM_B, RobotMap.mTray_Extend_ID);
    private static final Solenoid mTray_Retract = new Solenoid(RobotMap.mPCM_A, RobotMap.mTray_Retract_ID);
    private static final Solenoid mTalons_Hold = new Solenoid(RobotMap.mPCM_A, RobotMap.mTalons_Hold_ID);
    private static final Solenoid mTalons_Release = new Solenoid(RobotMap.mPCM_B, RobotMap.mTalons_Release_ID);
    private static final TalonSRX mShooter = new TalonSRX(RobotMap.mShooter_ID);

    public static final DigitalInput mCargo_Loaded_Sensor = new DigitalInput(RobotMap.mCargo_Loaded_Sensor_ID);
    public static final DigitalInput mHatch_Loaded_Sensor = new DigitalInput(RobotMap.mHatch_Loaded_Sensor_ID);

    public static final int CARGO_STATE_LOADED = 0;
    public static final int CARGO_STATE_UNLOADED = 1;
    public static int CARGO_STATE = CARGO_STATE_UNLOADED;

    public static final int TRAY_STATE_EXTENDED = 0;
    public static final int TRAY_STATE_RETRACTED = 1;
    public static int TRAY_STATE = TRAY_STATE_RETRACTED;

    public static final int TALON_STATE_HOLDING = 0;
    public static final int TALON_STATE_RELEASED = 1;
    public static int TALON_STATE = TALON_STATE_HOLDING;

    public static boolean WantHatch = false;

    public void Tray() {
        mShooter.setNeutralMode(NeutralMode.Brake);
        mShooter.configContinuousCurrentLimit(30);
        mShooter.configPeakCurrentLimit(0);
        mShooter.enableCurrentLimit(true);
        mShooter.setInverted(false);
    }

    public static void ShootCargo() {
        mShooter.set(ControlMode.PercentOutput, 1.0);
        SmartDashboard.putString("Shooting", "Ball");
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
        mTray_Extend.set(RobotMap.On);
        mTray_Retract.set(RobotMap.Off);
        TRAY_STATE = TRAY_STATE_EXTENDED;
    }
    public static void RetractTray() {
        mTray_Extend.set(RobotMap.Off);
        mTray_Retract.set(RobotMap.On);
        TRAY_STATE = TRAY_STATE_RETRACTED;
    }
    public static void TalonsHold() {
        mTalons_Release.set(RobotMap.Off);
        mTalons_Hold.set(RobotMap.On);
        TALON_STATE = TALON_STATE_HOLDING;
        SmartDashboard.putString("Talons holding", "");
    }
    public static void TalonsRelease() {
        mTalons_Hold.set(RobotMap.Off);
        mTalons_Release.set(RobotMap.On);
        TALON_STATE = TALON_STATE_RELEASED;
        SmartDashboard.putString("Talons releasng", "");
    }
    public static void TalonsAutoGrab() {
        while (WantHatch == true) {
            if (mHatch_Loaded_Sensor.get()) {
                TalonsHold();
                WantHatch = false;
                break;
            }
        }
    }
    public static void UpdateLoadState() {
        if (mCargo_Loaded_Sensor.get()) {
            CARGO_STATE = CARGO_STATE_LOADED;
        }
        else {
            CARGO_STATE = CARGO_STATE_UNLOADED;
        }
    }
    
    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      //  setDefaultCommand(new MySpecialCommand());
    }
}
