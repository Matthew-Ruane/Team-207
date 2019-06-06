/* package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;
import java.lang.Math.*;

public class PIDtest extends Command {

  private Timer timer;
  private int state;
  private int holding = 0;
  private int moving = 1;
  private boolean timerflag, firstrun;
  private double turn, maxOutput, maxOutputStep, maxOutputMax, prevleft, prevright, left, right, heading;
  
  public PIDtest() {
  }
  @Override
  protected void initialize() {
    Drivebase.zeroYaw();
    maxOutput = 0.0;
    maxOutputStep = 0.01;
    maxOutputMax = 0.7;
    prevleft = 1;
    heading = Drivebase.getYaw();
    Drivebase.PIDturn.setSetpoint(heading);
    Drivebase.PIDturn.setOutputRange(-0.65, 0.65);
    SmartDashboard.putNumber("heading", heading);
    Drivebase.PIDleft.setSetpoint(50000);
    Drivebase.PIDright.setSetpoint(50000);
    Drivebase.PIDleft.enable();
    Drivebase.PIDright.enable();
    Drivebase.PIDturn.enable();
    firstrun = true;
  }
  @Override
  protected void execute() {
    //TurnOutput = Drivebase.getTurnOutput();
    left = Drivebase.PIDleftOutput.getOutput();
    right = Drivebase.PIDrightOutput.getOutput();
    turn = Drivebase.PIDturnOutput.getOutput();
    if (turn >= right || turn >= left) {
        turn = (right+left)/2;
    }
    
    SmartDashboard.putNumber("leftsign", Math.signum(left));
    SmartDashboard.putNumber("prevleftsign", Math.signum(prevleft));
    if (!(Math.signum(left) == Math.signum(prevleft))) {
        maxOutput = 0.1;
        Drivebase.PIDturn.disable();
        turn = 0;
    }
    maxOutput += maxOutputStep;
    if (maxOutput >= maxOutputMax) {
        maxOutput = maxOutputMax;
    }
    SmartDashboard.putNumber("maxoutput", maxOutput);
    Drivebase.PIDleft.setOutputRange(-maxOutput, maxOutput);
    Drivebase.PIDright.setOutputRange(-maxOutput, maxOutput);
    Drivebase.pidDrive(left, right, turn);
    prevleft = left;
    prevright = right;
    SmartDashboard.putNumber("left4x", Drivebase.leftEncoder.get());
    SmartDashboard.putNumber("right4x", Drivebase.rightEncoder.get());
    SmartDashboard.putNumber("pidoutput", left);
    SmartDashboard.putNumber("pidturn", Drivebase.PIDturnOutput.getOutput());
  }
  @Override
  protected boolean isFinished() {
    return false;
  }
  @Override
  protected void end() {
    Drivebase.PIDturn.disable();
    Drivebase.PIDturn.reset();
    Drivebase.StopDrivetrain();
  }
  @Override
  protected void interrupted() {
  }
}
 */