/* 
** If you fuck with my source I'll kill you
*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
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
  
  public DriveDistanceCommand(double DesiredDistance) {
    timer = new Timer();
    distance = DesiredDistance;
    timerflag = Constants.Off;
    Drivebase.zeroLeftEncoder();
    Drivebase.zeroRightEncoder();
    LeftDistanceTarget = Drivebase.DistanceInchesToTicks(distance) + Math.abs(Drivebase.leftEncoderZero);
    RightDistanceTarget = Drivebase.DistanceInchesToTicks(distance) + Math.abs(Drivebase.rightEncoderZero);
    SmartDashboard.putNumber("lefttarget", LeftDistanceTarget);
    SmartDashboard.putNumber("righttarget", RightDistanceTarget);
  }
  @Override
  protected void initialize() {
    heading = Drivebase.getYaw();
    Drivebase.PIDturn.setSetpoint(heading);
    SmartDashboard.putNumber("setheading", heading);
    Drivebase.PIDturn.enable();
    state = moving;
  }
  @Override
  protected void execute() {
    TurnOutput = Drivebase.getTurnOutput();
    Drivebase.motionmagic(LeftDistanceTarget, RightDistanceTarget, TurnOutput);
  }
  @Override
  protected boolean isFinished() {
    displacement = Drivebase.getLeftEncoderTicks();
    SmartDashboard.putNumber("displacement", displacement);
    SmartDashboard.putNumber("leftdistance", LeftDistanceTarget);
    if (Drivebase.onTargetDistance(LeftDistanceTarget, displacement) == true && state == moving) {
      state = holding;
      return false;
    }
    else if (Drivebase.onTargetDistance(LeftDistanceTarget, displacement) == true && state == holding && timerflag == Constants.Off) {
      timer.start();
      timerflag = Constants.On;
      return false;
    }
    else if (Drivebase.onTargetDistance(LeftDistanceTarget, displacement) == true && state == holding && timer.get() >= 1.0) {
      timer.stop();
      timer.reset();
      timerflag = Constants.Off;
      return true;
    }
    else if (Drivebase.onTargetDistance(LeftDistanceTarget, displacement) == false && timer.get() > 1.0) {
      timer.reset();
      return false;
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
