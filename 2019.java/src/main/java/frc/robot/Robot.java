package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utility.*;
import frc.robot.Auto.AutoCommands.*;

public class Robot extends TimedRobot {
  
  private static OI m_oi;
  Drivebase drivebase = Drivebase.getInstance();
  Elevator elevator = Elevator.getInstance();
  Tray tray = Tray.getInstance();
  Rangefinder rangefinder = Rangefinder.getInstance();
  RingBuffer shiftbuffer = new RingBuffer(16, 0);
  
  Command autonomousCommand;
  SendableChooser<Command> autoProgram = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_oi = new OI();
    m_oi.registerControls();
    elevator.zeroElevatorEncoder();
    drivebase.setCoast();

    autoProgram.addDefault("PID tuning", new VelocityPIDtest());
    autoProgram.addObject("CommandA", new CommandA());

    SmartDashboard.putData("Selected Auto", autoProgram);
  }

  /**
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivebase.ReportData();
  }
  @Override
  public void disabledInit() {
    drivebase.StopDrivetrain();
    elevator.stopElevator();
    drivebase.resetEncoders();
    Constants.DesiredDistance = 0;
    Constants.DesiredHeading = 0;
    drivebase.setCoast();
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
    drivebase.setCoast();
    drivebase.resetPosition();
    autonomousCommand = autoProgram.getSelected();

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putData("PIDturn", Drivebase.PIDturn);
  }
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    drivebase.StopDrivetrain();
    drivebase.setBrake();
  }
  @Override
  public void teleopPeriodic() {
    drivebase.curvature(OI.getLeftThrottleInput(), OI.getRightSteeringInputInverted());
    Scheduler.getInstance().run();
    
  }
  @Override
  public void testPeriodic() {
  }
}
