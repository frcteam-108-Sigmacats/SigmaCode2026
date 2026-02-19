package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.SpinDexer;

/**
 * Passes a game piece to a partner robot at a specific location. Calculates the required shot angle
 * and velocity for the pass.
 */
public class PassToPartnerCommand extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final SpinDexer spinDexer;
  private final Translation2d partnerPosition;

  private boolean aimed = false;
  private boolean readyToPass = false;
  private double passStartTime = 0;

  /**
   * Creates a new PassToPartnerCommand.
   *
   * @param drive The drive subsystem
   * @param shooter The shooter subsystem
   * @param spinDexer The spin dexer subsystem
   * @param partnerPosition The position of the partner robot on the field
   */
  public PassToPartnerCommand(
      Drive drive, Shooter shooter, SpinDexer spinDexer, Translation2d partnerPosition) {

    this.drive = drive;
    this.shooter = shooter;
    this.spinDexer = spinDexer;
    this.partnerPosition = partnerPosition;

    addRequirements(shooter, spinDexer);
  }

  @Override
  public void initialize() {
    aimed = false;
    readyToPass = false;
    passStartTime = 0;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    // Calculate vector to partner
    Translation2d toPartner = partnerPosition.minus(robotPose.getTranslation());
    double distance = toPartner.getNorm();

    // Calculate required angles
    Rotation2d angleToPartner = new Rotation2d(toPartner.getX(), toPartner.getY());
    double turretAngle = angleToPartner.minus(robotPose.getRotation()).getDegrees();

    // Calculate pass parameters (lower velocity, flatter trajectory than scoring)
    double passVelocity = calculatePassVelocity(distance);
    double hoodAngle = calculatePassAngle(distance);

    if (!aimed) {
      // Aim turret and hood
      shooter.setTurretAngle(turretAngle);
      shooter.setHoodAngle(hoodAngle);

      if (shooter.turretAtTarget(turretAngle) && shooter.hoodAtTarget(hoodAngle)) {
        aimed = true;
      }
    } else if (!readyToPass) {
      // Spin up flywheels
      shooter.setFlywheelVelocity(passVelocity);

      if (shooter.atTargetVelocity(passVelocity)) {
        readyToPass = true;
        passStartTime = System.currentTimeMillis();
      }
    } else {
      // Execute the pass
      spinDexer.runKicker(12.0 * 0.6); // Gentler kick for passing
    }
  }

  @Override
  public void end(boolean interrupted) {
    spinDexer.stop();

    if (!interrupted) {
      shooter.setFlywheelVelocity(ShooterConstants.kIdleSpeed);
    } else {
      shooter.stop();
    }
  }

  @Override
  public boolean isFinished() {
    // Finish 0.3 seconds after starting the pass
    return readyToPass && (System.currentTimeMillis() - passStartTime) > 300;
  }

  /**
   * Calculates the required flywheel velocity for a pass at the given distance. Passes use lower
   * velocities than scoring shots.
   */
  private double calculatePassVelocity(double distance) {
    // Linear interpolation for pass velocity
    // Closer = slower, further = faster
    if (distance < 3.0) {
      return 1500; // Low velocity for short passes
    } else if (distance < 6.0) {
      return 1500 + (distance - 3.0) * 333; // Ramp up to 2500 RPM
    } else {
      return 2500 + (distance - 6.0) * 200; // Up to 3000 RPM for long passes
    }
  }

  /**
   * Calculates the required hood angle for a pass at the given distance. Passes use flatter
   * trajectories than scoring shots.
   */
  private double calculatePassAngle(double distance) {
    // Flatter angle for passes
    if (distance < 3.0) {
      return 20.0;
    } else if (distance < 6.0) {
      return 25.0;
    } else {
      return 30.0;
    }
  }

  /** Factory method for passing to a partner in a known zone. */
  public static PassToPartnerCommand toZone(
      Drive drive, Shooter shooter, SpinDexer spinDexer, PassZone zone, boolean isBlueAlliance) {

    Translation2d position = zone.getPosition(isBlueAlliance);
    return new PassToPartnerCommand(drive, shooter, spinDexer, position);
  }

  /** Common pass zones on the field. */
  public enum PassZone {
    CENTER_FIELD,
    WING,
    STAGE;

    public Translation2d getPosition(boolean isBlueAlliance) {
      if (isBlueAlliance) {
        switch (this) {
          case CENTER_FIELD:
            return new Translation2d(8.27, 4.0); // Center of field
          case WING:
            return new Translation2d(5.0, 7.0); // Wing area
          case STAGE:
            return new Translation2d(4.0, 4.0); // Near stage
          default:
            return new Translation2d();
        }
      } else {
        // Mirror for red alliance
        switch (this) {
          case CENTER_FIELD:
            return new Translation2d(8.27, 4.0);
          case WING:
            return new Translation2d(11.5, 7.0);
          case STAGE:
            return new Translation2d(12.5, 4.0);
          default:
            return new Translation2d();
        }
      }
    }
  }
}
