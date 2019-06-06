/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Drivebase;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.DriveDistanceCommand;
import frc.utility.*;

public class TestAuto1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TestAuto1() {

    Drivebase.resetEncoders();
    Drivebase.zeroYaw();

  }
}
