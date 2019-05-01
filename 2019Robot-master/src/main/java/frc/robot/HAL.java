package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.SPI;

public class HAL {

    public static Drivetrain drive = new Drivetrain();
    public static LimelightSubsystem limelight = new LimelightSubsystem();
    public static Turret turret = new Turret();
    public static FrontClimber frontClimber = new FrontClimber();
    public static BackClimber backClimber = new BackClimber();
    public static Elevator elevator = new Elevator();
    public static HatchPatchSubsystem hatch = new HatchPatchSubsystem();
    public static BallFeeder ballFeed = new BallFeeder();
    public static Shoulder shoulder = new Shoulder(); //relies on hatch, init hatch first
    public static Wrist wrist = new Wrist();
    public static HatchGroundIntake hatchGroundIntake = new HatchGroundIntake();
    public static AHRS navX = new AHRS(SPI.Port.kMXP);
}