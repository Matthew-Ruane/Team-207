package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HAL;
import frc.robot.OI;
import frc.robot.commands.*;
import frc.robot.RobotMap;


public class Turret extends Subsystem{
  
  private State state;

  public static enum State {
		STOPPED,
		MANUAL_OUTPUT_TURRET,
		MOTION_MAGIC
	}

  private void setState(State state) {
		this.state = state;
  }
  
  public void stop(){
    setState(State.STOPPED);
  }

  public TalonSRX turret = new TalonSRX(RobotMap.TURRET_PORT);

  public static enum TurretPreset {
    FRONT(0),

    RIGHT_ROCKET_BACK(38494),

    RIGHT(16226),
    LEFT(-16226),
    BACK(32452),
    BACK2(32452);

    public double turretPosition;

      private TurretPreset(double turretPosition) {
        this.turretPosition = turretPosition;
      }
  }

  public void configMotorController(int timeout){
    // P = 1
    turret.config_kP(0, 2, timeout);
    turret.config_kI(0, 0, timeout);
    turret.config_kD(0, 10, timeout);
    turret.config_kF(0, 0.08, timeout);
    turret.config_IntegralZone(0, 0, timeout);

    turret.configMotionAcceleration(3000, timeout);
    turret.configMotionCruiseVelocity(19000, timeout);
    turret.setNeutralMode(NeutralMode.Brake);

    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
    //feedArm.config_IntegralZone(0, Constants.intakeArmIZone, timeout);


    turret.setInverted(true);
    turret.setSensorPhase(true);
  }

  public Turret(){
    configMotorController(10);
  }
  
  public void periodic(){
    SmartDashboard.putNumber("Turret pos",getCurrentPosition());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HoldTurret(OI.turret));
    
  }
  public int getCurrentPosition() {
    return turret.getSelectedSensorPosition(0);
  }

  public boolean isAtPreset(TurretPreset preset) {
    return Math.abs(getCurrentPosition() - preset.turretPosition) < 5000;
  }

  public void setSpeed(double speed){
    turret.set(ControlMode.PercentOutput, speed);
  }

  public void setPosition(TurretPreset position){
    turret.set(ControlMode.MotionMagic, position.turretPosition);
  }

  public void setPosition(int position){
    turret.set(ControlMode.Position, position);
  }

  public void setPosition(double tickPos){
    turret.set(ControlMode.MotionMagic, tickPos);
  }
  public void setZero() {
    turret.setSelectedSensorPosition(0, 0, 10);
  }
  public void followLimelight()
  {
      setSpeed(HAL.limelight.xOffset()/27);
  }

  public void alignToTarget(){
    double ticksPerDegree = 32452/180;
    double tickPos = ticksPerDegree * (HAL.limelight.xOffset());
    setPosition(tickPos);
  }

  public void initialize() {
    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
  }
}
