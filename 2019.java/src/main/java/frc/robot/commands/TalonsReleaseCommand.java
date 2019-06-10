package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Tray;

public class TalonsReleaseCommand extends Command {
  private Tray tray = Tray.getInstance();

  public TalonsReleaseCommand() {
  }

  @Override
  protected void initialize() {
    if (tray.autograb == false) {
      tray.TalonsRelease();
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
