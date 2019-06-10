package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Tray;

public class ShootCargoCommand extends Command {

  private Timer shootTimer = new Timer();
  private boolean doneshooting = false;
  private Tray tray = Tray.getInstance();

  public ShootCargoCommand() {
  }

  @Override
  protected void initialize() {
    tray.ShootCargo();
    shootTimer.start();
  }

  @Override
  protected void execute() {
    if (shootTimer.get() >= 1.0) {
      shootTimer.stop();
      shootTimer.reset();
      tray.StopShootCargo();
      doneshooting = true;
    }
    else {
      doneshooting = false;
    }
  }

  @Override
  protected boolean isFinished() {
    if (doneshooting == true) {
      return true;
    }
    else {
      return false;
    }
  }
  
  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
