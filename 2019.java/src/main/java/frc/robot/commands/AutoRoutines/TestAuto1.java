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

public class TestAuto1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TestAuto1() {

    Drivebase.resetEncoders();
    Drivebase.zeroYaw();

    //addSequential(new DriveDistanceCommand(150));
    addSequential(new RotateToAngle(150));
    //addSequential(new DriveDistanceCommand(70));


    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
