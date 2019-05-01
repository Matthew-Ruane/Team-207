
package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.OI;
import frc.robot.commands.*;
import frc.robot.RobotMap;



public class FrontClimber extends Subsystem{

  public TalonSRX frontClimber = new TalonSRX(RobotMap.INTAKE_ARM_PORT);

  public static enum FrontClimberPreset {
    GROUND(2250),
    //PASS3(1000), 
    PASS3(931),
    PASS2(1800),
    TEST(400),
    HOME(1),
    AUTO_CLIMB(3477);

    public double frontClimberPosition;

      private FrontClimberPreset(double frontClimberPosition) {
        this.frontClimberPosition = frontClimberPosition;
      }
  }

  public void configMotorController(int timeout){
    // P = 1
    frontClimber.config_kP(0, 2, timeout);
    frontClimber.config_kI(0, 0, timeout);
    frontClimber.config_kD(0, 0, timeout);
    frontClimber.config_kF(0, 0, timeout);

    frontClimber.configMotionAcceleration(7500, timeout);
    frontClimber.configMotionCruiseVelocity(7500, timeout);
    frontClimber.setNeutralMode(NeutralMode.Brake);

    frontClimber.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
    //feedArm.config_IntegralZone(0, Constants.intakeArmIZone, timeout);

    frontClimber.setInverted(true);
    frontClimber.setSensorPhase(true);
  }

  public FrontClimber(){
    configMotorController(10);
  }

  public void periodic(){
    SmartDashboard.putNumber("Front Climber pos",getCurrentPosition());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new SetFrontClimber(OI.intake));
    
  }
  public int getCurrentPosition() {
    return frontClimber.getSelectedSensorPosition(0);
  }

  public double getCurrentPosition_double() {
    return frontClimber.getSelectedSensorPosition(0);
  }

  public void setSpeed(double speed){
    frontClimber.set(ControlMode.PercentOutput, speed);
  }

  public void setPosition(FrontClimberPreset position){
    frontClimber.set(ControlMode.MotionMagic, position.frontClimberPosition);
  }

  public void setPosition(int position){
    frontClimber.set(ControlMode.MotionMagic, position);
  }

  public void setPosition(double position){
    frontClimber.set(ControlMode.MotionMagic, position);
  }

  public void initialize() {
    frontClimber.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
  }
  public void setZero() {
    frontClimber.setSelectedSensorPosition(0, 0, 10);
  }
}
