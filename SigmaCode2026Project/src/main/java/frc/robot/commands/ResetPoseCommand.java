package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

/**
 * Resets the robot's pose to a known position. Can reset to origin, alliance starting position, or
 * custom pose.
 */
public class ResetPoseCommand extends InstantCommand {

  /** Reset modes for different scenarios. */
  public enum ResetMode {
    ORIGIN, // Reset to (0, 0, 0°)
    ALLIANCE_START, // Reset to alliance starting position
    CURRENT_ROTATION, // Reset position to origin but keep current rotation
    CUSTOM // Use custom pose
  }

  /**
   * Resets pose to origin (0, 0, 0°).
   *
   * @param drive The drive subsystem
   */
  public ResetPoseCommand(Drive drive) {
    this(drive, ResetMode.ORIGIN, new Pose2d());
  }

  /**
   * Resets pose using the specified mode.
   *
   * @param drive The drive subsystem
   * @param mode The reset mode
   */
  public ResetPoseCommand(Drive drive, ResetMode mode) {
    this(drive, mode, new Pose2d());
  }

  /**
   * Resets pose to a custom pose.
   *
   * @param drive The drive subsystem
   * @param customPose The custom pose to reset to
   */
  public ResetPoseCommand(Drive drive, Pose2d customPose) {
    this(drive, ResetMode.CUSTOM, customPose);
  }

  /** Internal constructor with all options. */
  private ResetPoseCommand(Drive drive, ResetMode mode, Pose2d customPose) {
    super(
        () -> {
          Pose2d targetPose;

          switch (mode) {
            case ORIGIN:
              targetPose = new Pose2d();
              break;

            case ALLIANCE_START:
              boolean isBlue =
                  DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                      == DriverStation.Alliance.Blue;
              targetPose =
                  isBlue
                      ? new Pose2d(1.5, 5.5, new Rotation2d()) // Blue starting position
                      : new Pose2d(15.0, 5.5, Rotation2d.fromDegrees(180)); // Red starting position
              break;

            case CURRENT_ROTATION:
              targetPose = new Pose2d(0, 0, drive.getPose().getRotation());
              break;

            case CUSTOM:
              targetPose = customPose;
              break;

            default:
              targetPose = new Pose2d();
              break;
          }

          drive.setPose(targetPose);
        },
        drive);
  }

  /** Factory method to reset to origin. */
  public static ResetPoseCommand toOrigin(Drive drive) {
    return new ResetPoseCommand(drive, ResetMode.ORIGIN);
  }

  /** Factory method to reset to alliance starting position. */
  public static ResetPoseCommand toAllianceStart(Drive drive) {
    return new ResetPoseCommand(drive, ResetMode.ALLIANCE_START);
  }

  /** Factory method to reset position but keep rotation. */
  public static ResetPoseCommand resetPositionOnly(Drive drive) {
    return new ResetPoseCommand(drive, ResetMode.CURRENT_ROTATION);
  }

  /** Factory method to reset to a specific pose. */
  public static ResetPoseCommand toCustomPose(Drive drive, Pose2d pose) {
    return new ResetPoseCommand(drive, pose);
  }
}
