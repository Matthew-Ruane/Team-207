/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.ninjaLib.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
/**
 * Add your docs here.
 */
public class HatchPatchSubsystem extends StatefulSubsystem<HatchPatchSubsystem.State> {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid soulPatch = new DoubleSolenoid(0,1);

  public enum State {
    OUT,
    IN
  }
  
  @Override
	public void setState(State state) {
		switch (state) {
			case OUT:
				soulPatch.set(DoubleSolenoid.Value.kReverse);
				break;
			default:
			case IN:
				soulPatch.set(DoubleSolenoid.Value.kForward);
				break;
		}
  }
  
  // public void periodic(){
  //   SmartDashboard.putState("Intake",getState());
  // }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
