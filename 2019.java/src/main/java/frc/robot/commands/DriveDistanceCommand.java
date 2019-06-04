/* 
** If you fuck with my source I'll kill you
*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;

public class DriveDistanceCommand extends Command {

  private Timer timer;
  private int state;
  private int holding = 0;
  private int moving = 1;
  private boolean timerflag, turn;
  private double distance, LeftDistanceTarget, RightDistanceTarget, displacement, heading, TurnOutput;
  
  public DriveDistanceCommand(double DesiredDistance, double DesiredHeading) {
    timer = new Timer();
    this.distance = DesiredDistance;
    this.heading = DesiredHeading;

  }
  @Override
  protected void initialize() {
    timerflag = Constants.Off;
    Drivebase.zeroLeftEncoder();
    Drivebase.zeroRightEncoder();
    LeftDistanceTarget = Drivebase.DistanceInchesToTicks(distance) + Math.abs(Drivebase.leftEncoderZero);
    RightDistanceTarget = Drivebase.DistanceInchesToTicks(distance) + Math.abs(Drivebase.rightEncoderZero);
    Drivebase.PIDturn.setSetpoint(heading);
    SmartDashboard.putNumber("setheading", heading);
    SmartDashboard.putNumber("this.distance", distance);
    SmartDashboard.putNumber("lefttarget", LeftDistanceTarget);
    SmartDashboard.putNumber("righttarget", RightDistanceTarget);
    Drivebase.PIDturn.enable();
    state = moving;
  }
  @Override
  protected void execute() {
    TurnOutput = Drivebase.getTurnOutput();
    displacement = Drivebase.getLeftEncoderTicks();
    Drivebase.motionmagic(LeftDistanceTarget, RightDistanceTarget, TurnOutput);
  }
  @Override
  protected boolean isFinished() {
    SmartDashboard.putNumber("displacement", displacement);
    SmartDashboard.putNumber("leftdistance", LeftDistanceTarget);
    if (Drivebase.onTargetDistance(Drivebase.DistanceInchesToTicks(distance), displacement) == true && timerflag == Constants.Off) {
      timer.start();
      timerflag = Constants.On;
      return false;
    }
    if (timer.get() > 0.3) {
      timer.stop();
      timer.reset();
      return true;
    }
    else {
      return false;
    }
  }
  @Override
  protected void end() {
    Drivebase.PIDturn.disable();
    Drivebase.PIDturn.reset();
    Drivebase.StopDrivetrain();
  }
  @Override
  protected void interrupted() {
/*     Drivebase.PIDturn.reset();
    cancel(); */
  }
}
