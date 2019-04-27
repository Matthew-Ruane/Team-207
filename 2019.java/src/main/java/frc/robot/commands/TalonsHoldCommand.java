package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Tray;

public class TalonsHoldCommand extends Command {
  public TalonsHoldCommand() {
  }

  @Override
  protected void initialize() {
    Tray.TalonsHold();
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
