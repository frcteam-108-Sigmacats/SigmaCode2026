package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Automatically aims the turret and hood at a target position on the field. Can optionally rotate
 * the robot to face the target as well.
 */
public class AutoAimCommand extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Translation2d targetPosition;
  private final boolean rotateRobot;

  private final PIDController turretController;
  private final PIDController hoodController;

  /**
   * Creates a new AutoAimCommand.
   *
   * @param drive The drive subsystem
   * @param shooter The shooter subsystem
   * @param targetPosition The target position on the field (meters)
   * @param rotateRobot Whether to rotate the robot to face the target
   */
  public AutoAimCommand(
      Drive drive, Shooter shooter, Translation2d targetPosition, boolean rotateRobot) {
    this.drive = drive;
    this.shooter = shooter;
    this.targetPosition = targetPosition;
    this.rotateRobot = rotateRobot;

    turretController = new PIDController(0.02, 0, 0);
    hoodController = new PIDController(0.05, 0, 0);

    turretController.setTolerance(1.0); // 1 degree tolerance
    hoodController.setTolerance(0.5); // 0.5 degree tolerance

    addRequirements(shooter);
    if (rotateRobot) {
      addRequirements(drive);
    }
  }

  @Override
  public void initialize() {
    turretController.reset();
    hoodController.reset();
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    // Calculate vector to target
    Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
    double distance = toTarget.getNorm();

    // Calculate required angles
    Rotation2d angleToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());
    double turretAngle = angleToTarget.minus(robotPose.getRotation()).getDegrees();

    // Simple ballistic calculation for hood angle
    double hoodAngle = calculateHoodAngle(distance);

    // Apply turret control
    double turretOutput = turretController.calculate(0, turretAngle);
    shooter.setTurretAngle(turretAngle);

    // Apply hood control
    shooter.setHoodAngle(hoodAngle);

    // Optionally rotate robot
    if (rotateRobot) {
      // This would integrate with drive system
      // For now, just ensure turret stays in valid range
      if (Math.abs(turretAngle) > 80) {
        // Rotate robot to bring turret back to center
        // Implementation depends on drive system
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Keep current aim unless interrupted
  }

  @Override
  public boolean isFinished() {
    return turretController.atSetpoint() && hoodController.atSetpoint();
  }

  /**
   * Calculates hood angle based on distance to target. This is a simplified ballistic calculation.
   *
   * @param distance Distance to target in meters
   * @return Hood angle in degrees
   */
  private double calculateHoodAngle(double distance) {
    // Simple linear interpolation
    // TODO: Replace with actual ballistic lookup table
    if (distance < 2.0) {
      return 30.0;
    } else if (distance < 4.0) {
      return 35.0 + (distance - 2.0) * 5.0;
    } else if (distance < 6.0) {
      return 45.0 + (distance - 4.0) * 5.0;
    } else {
      return 55.0;
    }
  }

  /**
   * Creates an auto aim command targeting the speaker.
   *
   * @param drive The drive subsystem
   * @param shooter The shooter subsystem
   * @param isBlueAlliance Whether on blue alliance
   * @return Auto aim command for speaker
   */
  public static AutoAimCommand aimAtSpeaker(Drive drive, Shooter shooter, boolean isBlueAlliance) {
    // Speaker positions (example - adjust based on field)
    Translation2d speakerPosition =
        isBlueAlliance
            ? new Translation2d(0.0, 5.5) // Blue speaker
            : new Translation2d(16.54, 5.5); // Red speaker

    return new AutoAimCommand(drive, shooter, speakerPosition, true);
  }

  /**
   * Creates an auto aim command targeting the amp.
   *
   * @param drive The drive subsystem
   * @param shooter The shooter subsystem
   * @param isBlueAlliance Whether on blue alliance
   * @return Auto aim command for amp
   */
  public static AutoAimCommand aimAtAmp(Drive drive, Shooter shooter, boolean isBlueAlliance) {
    // Amp positions (example - adjust based on field)
    Translation2d ampPosition =
        isBlueAlliance
            ? new Translation2d(1.8, 8.0) // Blue amp
            : new Translation2d(14.7, 8.0); // Red amp

    return new AutoAimCommand(drive, shooter, ampPosition, false);
  }
}
