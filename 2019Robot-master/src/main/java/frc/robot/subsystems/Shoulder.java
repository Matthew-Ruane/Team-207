package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.commands.*;
import frc.robot.OI;
import frc.robot.Constants;

import frc.robot.ninjaLib.Values;
import java.util.function.DoubleUnaryOperator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.HAL;

/**
 * Add your docs here.
 */
public class Shoulder extends Subsystem {
  private static final DoubleUnaryOperator gravityFunc = Values.mapRange(-125, -1250, -0.03, -0.235);
  public TalonSRX shoulder = new TalonSRX(RobotMap.SHOULDER_PORT);
    public static enum ShoulderPreset{

      // Up IS MORE NEGATIVE
      // DOWN IS MORE POSITIVE
      
      
      //////////////////
      // COMP PRESETS //
      //////////////////
      // UP IS MORE POSITIVE  
      // DOWN IS MORE NEGATIVE 
        GROUND_BALL(80), //was 3745+180-220-180
        GROUND_HATCH(0),
        TRAVEL(1218),
  
        PLACE_TRAVEL(1252),
  
        FEEDER_STATION_BALL(1218),
        FEEDER_STATION_HATCH(260), //was 278
      
        ROCKET_LOW_HATCH(238),
        ROCKET_MID_HATCH(1070),
        ROCKET_HIGH_HATCH(1055),
  
        CARGOSHIP_BALL(500),
  
        ROCKET_LOW_BALL(420),
        ROCKET_MID_BALL(1135),
        ROCKET_HIGH_BALL(1107),
      
         HATCH_GROUND_2_ELEVATOR_TRANSFER(617);
      
      //////////////////////////
      // NEW PRACTICE PRESETS //
      //////////////////////////
      // DOWN IS MORE NEGATIVE
      // UP IS MORE POSITIVE

        // GROUND_BALL(80+40), //was  287
        // GROUND_HATCH(0),
        // TRAVEL(1350),
  
        // PLACE_TRAVEL(1252),
  
        // FEEDER_STATION_BALL(1218),
        // //FEEDER_STATION_HATCH(218 + 85), //was 1601
      
        // FEEDER_STATION_HATCH(221+45),
        // ROCKET_LOW_HATCH(323),

        // ROCKET_MID_HATCH(1273),
        // ROCKET_HIGH_HATCH(1198), //was 1025
  
        // CARGOSHIP_BALL(500),
  
        // ROCKET_LOW_BALL(420),
        // ROCKET_MID_BALL(1135),
        // ROCKET_HIGH_BALL(1107),
      
  
        //  HATCH_GROUND_2_ELEVATOR_TRANSFER(617);

      ///////////////////////
      // PREACTICE PRESETS //
      ///////////////////////

      // DOWN IS MORE POSITIVE
      // UP IS MORE NEGATIVE

      // CARGOSHIP_BALL(960),
      // MAX_POS(100),
      
      // GROUND_BALL(2750+225+55), // was 1337
      // GROUND_HATCH(0),
      // TRAVEL(-181),
  
  
      // FEEDER_STATION_BALL(828),
      // FEEDER_STATION_HATCH(2575-175),
      
      // ROCKET_LOW_HATCH(1038+200+120+100+175+75+150+150+200),
      // ROCKET_MID_HATCH(711+200+100+100+25+175),
      // ROCKET_HIGH_HATCH(-705-150-175),
  
      // ROCKET_LOW_BALL(1313+200),
      // ROCKET_MID_BALL(1050+200),
      // ROCKET_HIGH_BALL(-450),
  
      // HATCH_GROUND_2_ELEVATOR_TRANSFER(-550+125+75+80+90+100+200);
  
  
      public double shoulderPosition;
      private ShoulderPreset(double shoulderPosition) {
          this.shoulderPosition = shoulderPosition;
        }
    }
      
  

  public void configMotorController(int timeout){
    //for empty/ball
    shoulder.config_kP(0, 4, timeout);
    shoulder.config_kI(0, 0, timeout);
    shoulder.config_kD(0, 55, timeout);
    shoulder.config_kF(0, 4.55, timeout);
    shoulder.config_IntegralZone(0, 0, timeout);

    shoulder.configMotionAcceleration(300, timeout); //was 400
    shoulder.configMotionCruiseVelocity(200, timeout); //was 400
    shoulder.setNeutralMode(NeutralMode.Brake);

    shoulder.setSensorPhase(true);
    shoulder.setInverted(true);

    shoulder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
    //feedArm.config_IntegralZone(0, Constants.intakeArmIZone, timeout);

  }

  public Shoulder(){
    configMotorController(10);

  }
  public void periodic(){
    SmartDashboard.putNumber("Shoulder pos", getCurrentPosition());
    SmartDashboard.putNumber("Shoulder vBus", shoulder.getMotorOutputVoltage());
    if(shoulder.getControlMode() == ControlMode.MotionMagic) {
      SmartDashboard.putNumber("Shoulder error", shoulder.getClosedLoopError());
      SmartDashboard.putNumber("Shoulder target", shoulder.getClosedLoopTarget());
    }
  }


  public int getCurrentPosition() {
    return shoulder.getSelectedSensorPosition(0);
  }

  public void setPosition(ShoulderPreset position){
    double gravity = 0;
    double hatchAdj = 0;

    /*
    if (HAL.hatch.getState() == HatchPatchSubsystem.State.OUT) {
      hatchAdj = -150;
    }
    */

    /*
    if(getCurrentPosition() <= -125) {
      gravity = gravityFunc.applyAsDouble(getCurrentPosition());
    }
    */
    shoulder.set(ControlMode.MotionMagic, position.shoulderPosition);
    //shoulder.set(ControlMode.MotionMagic, position.shoulderPosition + hatchAdj, DemandType.ArbitraryFeedForward, gravity);
  }
  public void setPosition(double pos){
    shoulder.set(ControlMode.MotionMagic, pos);
  }
  public void setSpeed(double speed){
    shoulder.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new SetShoulder(OI.shoulder));
  }

  public void setZero() {
    shoulder.setSelectedSensorPosition(0, 0, 10);
  }

  public boolean isAtTarget() {
    //100 ticks tolerance
    return Math.abs(getCurrentPosition() - shoulder.getClosedLoopTarget()) < 100;
  }

  void setOutput(double output) {
    shoulder.set(ControlMode.PercentOutput, output);
  }
  
  void stop() {
    shoulder.set(ControlMode.PercentOutput, 0);
  }
}