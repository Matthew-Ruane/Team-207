package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.SetElevator;
import frc.robot.commands.SetElevatorShoulderWrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {

  private int desiredElevatorPosition;
  
  public TalonSRX elevatorMaster =  new TalonSRX(RobotMap.ELEVATOR_MASTER_PORT);
  public VictorSPX elevatorSlave = new VictorSPX(RobotMap.ELEVATOR_SLAVE_PORT);

  public static enum ElevatorPreset{
    //////////////
    // COMP BOT //
    //////////////

    // UP IS MORE NEGATIVE
    CARGOSHIP_BALL(-11000+1000),
    TOP(100),
    
    GROUND_BALL(-34), // was 5
    GROUND_HATCH(0),
    TRAVEL(0),
    VISION_TRAVEL(-4300),

    FEEDER_STATION_BALL(0),
    FEEDER_STATION_HATCH(0),
    
    ROCKET_LOW_HATCH(0), //0

    ROCKET_MID_HATCH(-3878-500-1800), //-12220
    ROCKET_HIGH_HATCH(-18269+1000), //-18250

    ROCKET_LOW_BALL(-50),
    ROCKET_MID_BALL(-6460),
    ROCKET_HIGH_BALL(-18150),

    HATCH_GROUND_2_ELEVATOR_TRANSFER(-50);
    

    //////////////////
    // PRACTICE BOT //
    //////////////////
    // CARGOSHIP_BALL(-11000+100),
    // TOP(100),
    
    // GROUND_BALL(-44), // was 5
    // GROUND_HATCH(0),
    // TRAVEL(-5), //-4000 for ball feed at rocket

    // FEEDER_STATION_BALL(-44),
    // FEEDER_STATION_HATCH(-798),
    
    // ROCKET_LOW_HATCH(-200), //0
    // ROCKET_MID_HATCH(-10960+175+150+275), //-12220
    // ROCKET_HIGH_HATCH(-18150), //-18250

    // ROCKET_LOW_BALL(-50),
    // ROCKET_MID_BALL(-13446+100),
    // ROCKET_HIGH_BALL(-18100+125),

    // VISION_TRAVEL(-4300),

    // HATCH_GROUND_2_ELEVATOR_TRANSFER(-50);

    public double elevatorPosition;
			
		private ElevatorPreset(double elevatorPosition) {
			this.elevatorPosition = elevatorPosition;
		}
  }

  public void configMotorController(int timeout){
    elevatorMaster.config_kP(0, 1.25, timeout); //was 1.2
    elevatorMaster.config_kI(0, 0.0, timeout); // was 0
    elevatorMaster.config_kD(0, 1.0, timeout); //was 0
    elevatorMaster.config_kF(0, 0.18, timeout); //was .0
    elevatorMaster.config_IntegralZone(0, 0, timeout);

    elevatorMaster.configMotionAcceleration(6000, timeout); //was 10000
    elevatorMaster.configMotionCruiseVelocity(10000, timeout); //was 12000
    elevatorMaster.setNeutralMode(NeutralMode.Brake);

    elevatorMaster.setInverted(false);
    elevatorSlave.setInverted(false);
    elevatorMaster.setSensorPhase(false);
    elevatorSlave.setSensorPhase(false);

    elevatorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
    //feedArm.config_IntegralZone(0, Constants.intakeArmIZone, timeout);

  }
  public void periodic(){
    SmartDashboard.putNumber("Elevator pos",getCurrentPosition());
    if(elevatorMaster.getControlMode() == ControlMode.MotionMagic) {
      SmartDashboard.putNumber("Elevator error", elevatorMaster.getClosedLoopError());
      SmartDashboard.putNumber("Elevator target", elevatorMaster.getClosedLoopTarget());
    }
  }
  public Elevator(){
    elevatorSlave.follow(elevatorMaster); 
    configMotorController(10);
  }


  public int getCurrentPosition() {
    return elevatorMaster.getSelectedSensorPosition(0);
  }

  public void setPosition(ElevatorPreset position){
    elevatorMaster.set(ControlMode.MotionMagic, position.elevatorPosition);
  }

  public boolean isAtTarget() {
    //300 ticks tolerance
    return Math.abs(getCurrentPosition() - elevatorMaster.getClosedLoopTarget()) < 300;
  }

  public void setPosition(double tickPos)
  {
    elevatorMaster.set(ControlMode.MotionMagic, tickPos);
  }

/* 
  public int elevatorNudgeUp(){
    return elevatorMaster.getSelectedSensorPosition(0) + 50;
  }

  public int elevatorNudgeDown(){
    return elevatorMaster.getSelectedSensorPosition(0) - 50;
  }
*/

  public void setSpeed(double speed){
    elevatorMaster.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new SetElevator(OI.elevator));
  }

  public void setZero() {
    elevatorMaster.setSelectedSensorPosition(0, 0, 10);
  }

  void setOutput(double output) {
    elevatorMaster.set(ControlMode.PercentOutput, output);
  }
  
  void stop() {
    elevatorMaster.set(ControlMode.PercentOutput, 0);
  }
}