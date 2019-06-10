package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Tray;
import frc.robot.Constants;

public class TrayExtensionToggle extends Command {
  private Tray tray = Tray.getInstance();
  public TrayExtensionToggle() {
  }

  @Override
  protected void initialize() {
    if (Constants.TRAY_STATE == Constants.TRAY_STATE_EXTENDED) {
      tray.RetractTray();
    }
    else if (Constants.TRAY_STATE == Constants.TRAY_STATE_RETRACTED) {
      tray.ExtendTray();
    }
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
