package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoRoutines.*;

public class Robot extends TimedRobot {
  
  private static OI m_oi;
  Drivebase drive = Drivebase.getInstance();
  Elevator elevator = Elevator.getInstance();
  Tray tray = Tray.getInstance();
  Rangefinder rangefinder = Rangefinder.getInstance();
  

  Command autonomousCommand;
  SendableChooser<Command> autoProgram = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_oi.registerControls();
    Elevator.zeroElevatorEncoder();
    Drivebase.DisableVoltComp();
    Drivebase.setCoast();

    autoProgram.addDefault("TestAuto1", new TestAuto1());
    autoProgram.addObject("TestAuto2", new TestAuto1());

    SmartDashboard.putData("Selected Auto", autoProgram);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Elevator.stopElevator();
    Drivebase.resetEncoders();
    Constants.DesiredDistance = 0;
    Constants.DesiredHeading = 0;
    Drivebase.setCoast();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = autoProgram.getSelected();

    
    /*   String autoSelected = SmartDashboard.getString("Auto Selector",
      "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
      = new MyAutoCommand(); break; case "Default Auto": default:
      autonomousCommand = new ExampleCommand(); break; } */
     

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    Drivebase.setBrake();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Drivebase.curvature(OI.getLeftThrottleInput(), OI.getRightSteeringInput());
    Drivebase.ReportData();
    Scheduler.getInstance().run();
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
