/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.RobotMap;
/**
 * Add your docs here.
 */
public class UltrasonicSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Ultrasonic ultrasonic = new Ultrasonic(RobotMap.ULTRA_PING_CHANNEL, RobotMap.ULTRA_ECHO_CHANNEL);

  public double getDistance(){
      return ultrasonic.getRangeInches();
  }

  public Ultrasonic.Unit getDistancePID()
  {
      return ultrasonic.getDistanceUnits();
  }

  public double getPID(){
    return ultrasonic.pidGet();
  }

  public boolean enabled(){
    return ultrasonic.isEnabled();
  }
  
  public boolean inRange(){
    return ultrasonic.isRangeValid();
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
