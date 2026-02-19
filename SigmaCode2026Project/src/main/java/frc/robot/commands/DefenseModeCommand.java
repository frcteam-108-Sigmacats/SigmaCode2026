package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * Puts the robot in X-stance (defense mode) to resist pushing. All wheels are oriented at 45-degree
 * angles forming an X pattern. This makes it very difficult for other robots to push this robot.
 */
public class DefenseModeCommand extends Command {
  private final Drive drive;

  /**
   * Creates a new DefenseModeCommand.
   *
   * @param drive The drive subsystem
   */
  public DefenseModeCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.stopWithX();
  }

  @Override
  public void execute() {
    // Continuously maintain X-stance
    drive.stopWithX();
  }

  @Override
  public void end(boolean interrupted) {
    // Don't need to do anything special - next command will take over
  }

  @Override
  public boolean isFinished() {
    // Never finishes on its own - must be interrupted
    return false;
  }
}
