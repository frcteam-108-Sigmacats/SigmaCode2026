package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.SpinDexer;

/**
 * Complete autonomous scoring sequence: 1. Drive to scoring position 2. Auto aim at target 3. Shoot
 * sequence 4. Return to position or continue
 */
public class AutoScoreCommand extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final SpinDexer spinDexer;
  private final ScoringTarget target;

  private Command sequenceCommand;

  /** Scoring target options. */
  public enum ScoringTarget {
    SPEAKER,
    AMP
  }

  /**
   * Creates a new AutoScoreCommand.
   *
   * @param drive The drive subsystem
   * @param shooter The shooter subsystem
   * @param spinDexer The spin dexer subsystem
   * @param target The target to score in (SPEAKER or AMP)
   */
  public AutoScoreCommand(Drive drive, Shooter shooter, SpinDexer spinDexer, ScoringTarget target) {
    this.drive = drive;
    this.shooter = shooter;
    this.spinDexer = spinDexer;
    this.target = target;

    addRequirements(drive, shooter, spinDexer);
  }

  @Override
  public void initialize() {
    boolean isBlueAlliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;

    // Build the command sequence based on target
    if (target == ScoringTarget.SPEAKER) {
      sequenceCommand =
          Commands.sequence(
              // Drive to speaker scoring position
              DriveToPoseCommand.toScoringPosition(
                  drive, DriveToPoseCommand.ScoringPosition.SPEAKER_CENTER, isBlueAlliance),

              // Auto aim at speaker
              AutoAimCommand.aimAtSpeaker(drive, shooter, isBlueAlliance).withTimeout(2.0),

              // Execute shoot sequence
              ShootSequenceCommand.forSpeaker(shooter, spinDexer));
    } else {
      sequenceCommand =
          Commands.sequence(
              // Drive to amp scoring position
              DriveToPoseCommand.toScoringPosition(
                  drive, DriveToPoseCommand.ScoringPosition.AMP, isBlueAlliance),

              // Auto aim at amp
              AutoAimCommand.aimAtAmp(drive, shooter, isBlueAlliance).withTimeout(1.0),

              // Execute shoot sequence
              ShootSequenceCommand.forAmp(shooter, spinDexer));
    }

    sequenceCommand.initialize();
  }

  @Override
  public void execute() {
    if (sequenceCommand != null) {
      sequenceCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (sequenceCommand != null) {
      sequenceCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return sequenceCommand != null && sequenceCommand.isFinished();
  }

  /** Factory method to create a speaker scoring command. */
  public static AutoScoreCommand scoreSpeaker(Drive drive, Shooter shooter, SpinDexer spinDexer) {
    return new AutoScoreCommand(drive, shooter, spinDexer, ScoringTarget.SPEAKER);
  }

  /** Factory method to create an amp scoring command. */
  public static AutoScoreCommand scoreAmp(Drive drive, Shooter shooter, SpinDexer spinDexer) {
    return new AutoScoreCommand(drive, shooter, spinDexer, ScoringTarget.AMP);
  }
}
