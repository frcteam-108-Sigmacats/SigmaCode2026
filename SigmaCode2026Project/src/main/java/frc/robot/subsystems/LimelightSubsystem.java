package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.Set;

/**
 * Subsystem wrapping the Limelight 2+ / 3 / 3G camera.
 *
 * <p>Responsibilities:
 *
 * <ul>
 *   <li>Provide a valid robot-pose estimate for the swerve pose estimator.
 *   <li>Report Limelight target information (TX, TY, tag ID) to commands.
 *   <li>Allow commands to filter which AprilTag IDs are considered valid targets.
 * </ul>
 *
 * <p>FIX: The original implementation was a stub that always returned {@code false} / {@code new
 * Pose2d()} / {@code 0.0}. This implementation uses {@link LimelightHelpers} (already present in
 * the project) to query the real camera over NetworkTables.
 */
public class LimelightSubsystem extends SubsystemBase {

  /** NetworkTables name of the Limelight. Change if you renamed it in the web UI. */
  private static final String LIMELIGHT_NAME = "limelight";

  /**
   * Minimum tag area (% of image) below which pose estimates are discarded as too noisy. Increase
   * if you see erratic pose jumps from far-away tags.
   */
  private static final double MIN_TAG_AREA = 0.1;

  /** Optional allow-list set by the currently running command. null = accept any tag. */
  private Set<Integer> allowedTagIds = null;

  // Cached values refreshed each periodic() call
  private PoseEstimate latestPoseEstimate = null;
  private boolean hasValidPose = false;
  private boolean hasAnyTarget = false;
  private double txDegrees = 0.0;
  private int primaryTagId = -1;

  /** Default no-arg constructor (used by RobotContainer). */
  public LimelightSubsystem() {}

  /** Secondary constructor kept for backwards compatibility (string param unused). */
  public LimelightSubsystem(String name) {}

  // ── Periodic ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // Choose the correct pose estimator origin depending on alliance colour.
    // wpiBlue uses the WPILib field coordinate convention (blue origin at bottom-left).
    boolean isBlueAlliance =
        DriverStation.getAlliance()
            .map(a -> a == Alliance.Blue)
            .orElse(true); // default to blue coordinate frame if unknown

    PoseEstimate estimate =
        isBlueAlliance
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME)
            : LimelightHelpers.getBotPoseEstimate_wpiRed(LIMELIGHT_NAME);

    // Validate estimate
    if (estimate != null && estimate.tagCount > 0 && estimate.avgTagArea >= MIN_TAG_AREA) {
      latestPoseEstimate = estimate;
      hasValidPose = true;
    } else {
      hasValidPose = false;
    }

    // Cache raw target info
    hasAnyTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
    txDegrees = LimelightHelpers.getTX(LIMELIGHT_NAME);
    primaryTagId = (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
  }

  // ── Pose estimation (for SwerveSubsystem) ────────────────────────────────

  /**
   * @return {@code true} if the Limelight has a fresh, reliable pose estimate that the swerve pose
   *     estimator should fuse.
   */
  public boolean hasValidMeasurement() {
    return hasValidPose;
  }

  /**
   * @return the latest robot-pose estimate in WPILib field coordinates. Only call this when {@link
   *     #hasValidMeasurement()} returns {@code true}.
   */
  public Pose2d getRobotPose() {
    return (latestPoseEstimate != null) ? latestPoseEstimate.pose : new Pose2d();
  }

  /**
   * @return the capture timestamp (FPGA time, seconds) of the latest pose estimate, compensated for
   *     camera latency. Pass this directly to {@code
   *     SwerveDrivePoseEstimator.addVisionMeasurement()}.
   */
  public double getTimestampSeconds() {
    return (latestPoseEstimate != null) ? latestPoseEstimate.timestampSeconds : 0.0;
  }

  // ── Target info (for TrackHubCommand) ────────────────────────────────────

  /**
   * @return {@code true} if the Limelight sees at least one AprilTag.
   */
  public boolean hasTarget() {
    return hasAnyTarget;
  }

  /**
   * @return horizontal offset from crosshair to target (degrees). Positive = target is to the right
   *     of centre.
   */
  public double getTX() {
    return txDegrees;
  }

  /**
   * @return the AprilTag ID of the primary (largest) target, or -1 if none.
   */
  public int getTargetId() {
    return primaryTagId;
  }

  // ── Tag filtering (for TrackHubCommand) ──────────────────────────────────

  /**
   * Restrict valid targets to a specific set of AprilTag IDs. Called by {@link
   * frc.robot.commands.TrackHubCommand} on initialize.
   *
   * @param tagIds set of allowed tag IDs; null or empty = accept any tag
   */
  public void setTargetTags(Set<Integer> tagIds) {
    this.allowedTagIds = (tagIds != null && !tagIds.isEmpty()) ? tagIds : null;
    applyTagFilter();
  }

  /**
   * Remove any tag filter previously applied by {@link #setTargetTags}. Called by {@link
   * frc.robot.commands.TrackHubCommand} on end.
   */
  public void clearTargetTagFilter() {
    this.allowedTagIds = null;
    // Pass an empty array to LimelightHelpers to restore "accept all"
    LimelightHelpers.SetFiducialIDFiltersOverride(LIMELIGHT_NAME, new int[0]);
  }

  /** Push the current allowed-tag list down to the Limelight via NT. */
  private void applyTagFilter() {
    if (allowedTagIds == null) {
      LimelightHelpers.SetFiducialIDFiltersOverride(LIMELIGHT_NAME, new int[0]);
    } else {
      int[] ids = allowedTagIds.stream().mapToInt(Integer::intValue).toArray();
      LimelightHelpers.SetFiducialIDFiltersOverride(LIMELIGHT_NAME, ids);
    }
  }
}
