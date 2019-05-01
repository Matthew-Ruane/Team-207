package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.*;
import frc.robot.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class BackClimber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public TalonSRX tailMaster =  new TalonSRX(RobotMap.TURTLE_MASTER_PORT);
  //public VictorSPX tailSlave = new VictorSPX(RobotMap.TURTLE_SLAVE_PORT);

  public static enum BackClimberPreset
  { 
      // Values get more negative from starting configuration
      ZERO(0),
      TEST(-500),
      NINETY(1192), 
      ONEEIGHTY(400),
      AUTO_CLIMB(1250); //was 1299

      public double backClimb;
    
      private BackClimberPreset(double backClimb) {
        this.backClimb = backClimb;
      }
  }

  public void configMotorController(int timeout){
    tailMaster.config_kP(0, 1, timeout);
    tailMaster.config_kI(0, 0, timeout);
    tailMaster.config_kD(0, 0, timeout);
    tailMaster.config_kF(0, 0, timeout);

    tailMaster.configMotionAcceleration(2200, timeout);
    tailMaster.configMotionCruiseVelocity(2200, timeout);
    tailMaster.setNeutralMode(NeutralMode.Brake);

    tailMaster.setSensorPhase(false); // true for comp bot

    tailMaster.setInverted(false);
    //tailSlave.setInverted(false);

    
    
    tailMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
  }


  public BackClimber(){
    //tailSlave.follow(tailMaster); 
    //tailMaster.set(ControlMode.PercentOutput, 1);
    //tailMaster.set(ControlMode.PercentOutput, 0.01);
    configMotorController(10);
  }

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new Climb(OI.tail,OI.intake));
    setDefaultCommand(new SetBackClimber(OI.tail));
    
  }
  public void setSpeed(double speed){
    tailMaster.set(ControlMode.PercentOutput, speed);
    //tailSlave.set(ControlMode.PercentOutput, speed);
  }
  public void setPosition(double pos){
    tailMaster.set(ControlMode.MotionMagic, pos);
  }

  public void setPosition(BackClimberPreset position){
    tailMaster.set(ControlMode.MotionMagic, position.backClimb);
    //tailMaster.set(ControlMode.PercentOutput,1);
    //tailMaster.set(ControlMode.PercentOutput, 1);
  }

  public void periodic(){
    SmartDashboard.putNumber("Back Climber",getCurrentPosition());
  }

  public int getCurrentPosition() {
    return tailMaster.getSelectedSensorPosition(0);
  }

  public double getCurrentPosition_double() {
    return tailMaster.getSelectedSensorPosition(0);
  }

  public void setZero() {
    tailMaster.setSelectedSensorPosition(0, 0, 10);
  }

  void setOutput(double output) {
    tailMaster.set(ControlMode.PercentOutput, output);
  }
  
  void stop() {
    tailMaster.set(ControlMode.PercentOutput, 0);
  }
}