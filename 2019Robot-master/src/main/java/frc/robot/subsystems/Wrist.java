package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.RobotMap;

import frc.robot.commands.*;
import frc.robot.OI;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class Wrist extends Subsystem {
  
  public TalonSRX wrist = new TalonSRX(RobotMap.WRIST_PORT);

  public static enum WristPreset {

    //////////////
    // COMP BOT //
    //////////////
    
    // DOWN IS MORE POSITIVE
    // UP IS MORE NEGATIVE
    MAX_POS(100),
    TRAVEL(1860),

    PLACE_TRAVEL(1860),

    GROUND2TRAVEL(1200),
    GROUND_BALL(1344+160+180), // was 1840-80-130-150 
    GROUND_HATCH(0),

    FEEDER_STATION_BALL(900-20),

    FEEDER_STATION_HATCH(772), //was 725 added 475
    ROCKET_LOW_HATCH(772), //942

    ROCKET_MID_HATCH(1600), //975
    ROCKET_HIGH_HATCH(1500), //1482
    
    ROCKET_LOW_BALL(1000), //was 1600
    ROCKET_MID_BALL(1825),
    ROCKET_HIGH_BALL(1650),


    HATCH_GROUND_2_ELEVATOR_TRANSFER(1890),
    CARGOSHIP_BALL(1850-20);

    //////////////////////////
    // NEW PRACTICE PRESETS //
    //////////////////////////

    // DOWN IS MORE POSITIVE
    // UP IS MORE NEGATIVE
    // MAX_POS(100),
    // TRAVEL(1700), //was 1860

    // PLACE_TRAVEL(1860),

    // GROUND2TRAVEL(1200),
    // GROUND_BALL(1484), // was   1851
    // GROUND_HATCH(0),

    // FEEDER_STATION_BALL(900-20),

    // //FEEDER_STATION_HATCH(772-100+75+80), //was 725 added 475
    // FEEDER_STATION_HATCH(513+40+50+30+40), 
    // ROCKET_LOW_HATCH(772-75), //942

    // ROCKET_MID_HATCH(1833-120), //975
    // ROCKET_HIGH_HATCH(1833-120-100-70), //1833-120
    
    // ROCKET_LOW_BALL(1000-70), //was 1600
    // ROCKET_MID_BALL(1825-70),
    // ROCKET_HIGH_BALL(1650),


    // HATCH_GROUND_2_ELEVATOR_TRANSFER(1890),
    // CARGOSHIP_BALL(1850-20);


    //////////////////
    // PRACTICE BOT //
    //////////////////

    // DOWN IS MORE POSITIVE
    // UP IS MORE NEGATIVE
    // MAX_POS(100),
    // TRAVEL(600),
    // GROUND_BALL(1840), // was 1957
    // GROUND_HATCH(0),

    // GROUND2TRAVEL(1200),

    // FEEDER_STATION_BALL(1035+90),
    // FEEDER_STATION_HATCH(1015+105),
    
    // ROCKET_LOW_HATCH(1112-50-100), //942
    // ROCKET_MID_HATCH(1065-50+100), //975
    // ROCKET_HIGH_HATCH(1512+80+150), //1482

    // ROCKET_LOW_BALL(1350),
    // ROCKET_MID_BALL(1490),
    // ROCKET_HIGH_BALL(1330+200+65),

    // HATCH_GROUND_2_ELEVATOR_TRANSFER(1724+100+150-75-60),

    // CARGOSHIP_BALL(1650+100);
    
    public double wristPosition;
			
			private WristPreset(double wristPosition) {
        this.wristPosition = wristPosition;
        
			}
  }
  public void configMotorController(int timeout){
    wrist.config_kP(0, 1.5, timeout);
    wrist.config_kI(0, 0, timeout);
    wrist.config_kD(0, 1.0, timeout);
    wrist.config_kF(0, 2.62, timeout);

    wrist.configMotionAcceleration(400, timeout);
    wrist.configMotionCruiseVelocity(300, timeout); 
    wrist.setNeutralMode(NeutralMode.Brake);

    wrist.setSensorPhase(true);
    wrist.setInverted(true);

    wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
    //feedArm.config_IntegralZone(0, Constants.intakeArmIZone, timeout);

  }

  public Wrist(){
    configMotorController(10);
  }
  public void periodic(){
    SmartDashboard.putNumber("Wrist pos",getCurrentPosition());
    if(wrist.getControlMode() == ControlMode.MotionMagic) {
      SmartDashboard.putNumber("Wrist error", wrist.getClosedLoopError());
      SmartDashboard.putNumber("Wrist target", wrist.getClosedLoopTarget());
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new ValueLogger());
    setDefaultCommand(new SetWrist(OI.wrist));

  }

  public int getCurrentPosition() {
    return wrist.getSelectedSensorPosition(0);
  }

  public boolean isAtTarget() {
    //100 ticks tolerance
    return Math.abs(getCurrentPosition() - wrist.getClosedLoopTarget()) < 100;
  }

  public void setPosition(WristPreset position){
    wrist.setInverted(true);

    wrist.set(ControlMode.MotionMagic, position.wristPosition);
  }

  public void setSpeed(double speed){
    wrist.set(ControlMode.PercentOutput, speed);
  }

  public void setZero() {
    wrist.setSelectedSensorPosition(0, 0, 10);
  }

  void setOutput(double output) {
    wrist.set(ControlMode.PercentOutput, output);
  }
  
  void stop() {
    wrist.set(ControlMode.PercentOutput, 0);
  }
}


