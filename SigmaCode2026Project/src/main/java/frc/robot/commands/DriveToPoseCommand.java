package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;

/** Drives the robot to a target pose using PathPlanner's pathfinding. */
public class DriveToPoseCommand extends Command {
  private final Drive drive;
  private final Pose2d targetPose;
  private final PathConstraints constraints;
  private Command pathfindCommand;

  /**
   * Creates a new DriveToPoseCommand.
   *
   * @param drive The drive subsystem
   * @param targetPose The target pose to drive to
   */
  public DriveToPoseCommand(Drive drive, Pose2d targetPose) {
    this(
        drive,
        targetPose,
        new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared));
  }

  /**
   * Creates a new DriveToPoseCommand with custom constraints.
   *
   * @param drive The drive subsystem
   * @param targetPose The target pose to drive to
   * @param constraints Path constraints (max speed, acceleration, etc.)
   */
  public DriveToPoseCommand(Drive drive, Pose2d targetPose, PathConstraints constraints) {
    this.drive = drive;
    this.targetPose = targetPose;
    this.constraints = constraints;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Build pathfinding command using PathPlanner
    pathfindCommand =
        AutoBuilder.pathfindToPose(
            targetPose, constraints, 0.0 // Goal end velocity
            );

    pathfindCommand.initialize();
  }

  @Override
  public void execute() {
    if (pathfindCommand != null) {
      pathfindCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathfindCommand != null) {
      pathfindCommand.end(interrupted);
    }
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    if (pathfindCommand == null) {
      return true;
    }
    return pathfindCommand.isFinished();
  }

  /**
   * Creates a command to drive to a scoring position.
   *
   * @param drive The drive subsystem
   * @param scoringPosition The scoring position (Speaker, Amp, etc.)
   * @param isBlueAlliance Whether on blue alliance
   * @return Drive to pose command
   */
  public static DriveToPoseCommand toScoringPosition(
      Drive drive, ScoringPosition scoringPosition, boolean isBlueAlliance) {

    Pose2d targetPose = scoringPosition.getPose(isBlueAlliance);
    return new DriveToPoseCommand(drive, targetPose);
  }

  /** Enum for common scoring positions on the field. */
  public enum ScoringPosition {
    SPEAKER_CENTER,
    SPEAKER_LEFT,
    SPEAKER_RIGHT,
    AMP;

    public Pose2d getPose(boolean isBlueAlliance) {
      // Example positions - adjust based on actual field measurements
      if (isBlueAlliance) {
        switch (this) {
          case SPEAKER_CENTER:
            return new Pose2d(1.5, 5.5, new edu.wpi.first.math.geometry.Rotation2d(0));
          case SPEAKER_LEFT:
            return new Pose2d(
                1.5, 6.5, new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(15)));
          case SPEAKER_RIGHT:
            return new Pose2d(
                1.5, 4.5, new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(-15)));
          case AMP:
            return new Pose2d(
                1.8, 8.0, new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(-90)));
          default:
            return new Pose2d();
        }
      } else {
        // Red alliance (mirrored)
        switch (this) {
          case SPEAKER_CENTER:
            return new Pose2d(15.0, 5.5, new edu.wpi.first.math.geometry.Rotation2d(Math.PI));
          case SPEAKER_LEFT:
            return new Pose2d(
                15.0, 4.5, new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(165)));
          case SPEAKER_RIGHT:
            return new Pose2d(
                15.0, 6.5, new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(-165)));
          case AMP:
            return new Pose2d(
                14.7, 8.0, new edu.wpi.first.math.geometry.Rotation2d(Math.toRadians(-90)));
          default:
            return new Pose2d();
        }
      }
    }
  }
}
