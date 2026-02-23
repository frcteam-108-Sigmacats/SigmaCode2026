package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * Continuously aims the turret, spins the shooter wheels, and positions the hood based on a target
 * distance supplied externally (e.g. from a vision pipeline or a fixed pre-match setpoint).
 *
 * <p>This command is self-contained: it requires only the {@link Shooter} subsystem and does
 * <em>not</em> depend on the drive subsystem. Robot-relative aiming is handled by passing in a
 * {@code targetAngleSupplier} and {@code distanceSupplier} from whichever source is available
 * (vision, LiDAR, driver override, etc.).
 *
 * <p>Typical usage with a fixed angle + distance (e.g. auto pre-aim):
 *
 * <pre>
 *   new TrackHubCommand(turret, () -> 0.0, () -> 3.0)
 * </pre>
 *
 * <p>Typical usage wired to a vision system:
 *
 * <pre>
 *   new TrackHubCommand(turret, vision::getTurretAngleRad, vision::getDistanceMeters)
 * </pre>
 */
public class TrackHubCommand extends Command {

  private final Shooter turret;

  /**
   * Supplies the desired turret angle in radians relative to the robot's forward axis. Positive =
   * counterclockwise when viewed from above. Return 0.0 to keep the turret facing forward.
   */
  private final java.util.function.DoubleSupplier targetAngleRadSupplier;

  /**
   * Supplies the estimated distance to the target in metres. Used to interpolate shooter RPM and
   * hood angle from {@link ShooterConstants.ShooterStates}.
   */
  private final java.util.function.DoubleSupplier distanceMetersSupplier;

  /** Wheel diameter in metres used to convert RPM -> surface speed (m/s). */
  private static final double WHEEL_DIAMETER_M = 2.0 * ShooterConstants.shooterWheelRadiusMeters;

  /**
   * @param turret the turret subsystem
   * @param targetAngleRadSupplier robot-relative turret angle in radians (0 = forward)
   * @param distanceMetersSupplier distance to the target in metres for lookup-table interpolation
   */
  public TrackHubCommand(
      Shooter turret,
      java.util.function.DoubleSupplier targetAngleRadSupplier,
      java.util.function.DoubleSupplier distanceMetersSupplier) {
    this.turret = turret;
    this.targetAngleRadSupplier = targetAngleRadSupplier;
    this.distanceMetersSupplier = distanceMetersSupplier;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    // Nothing to initialise – command calculates fresh each cycle
  }

  @Override
  public void execute() {
    double angleRad = targetAngleRadSupplier.getAsDouble();
    double distMeters = distanceMetersSupplier.getAsDouble();

    double rpm = interpolate(distMeters, ShooterConstants.ShooterStates.shooterRPMMap);
    double hoodDeg = interpolate(distMeters, ShooterConstants.ShooterStates.shooterHoodAngleMap);
    double surfaceMps = (rpm / 60.0) * Math.PI * WHEEL_DIAMETER_M;

    turret.setTurretAngle(new Rotation2d(angleRad));
    turret.setShooterSpeed(surfaceMps);
    turret.setHoodAngle(hoodDeg);

    Logger.recordOutput("TrackHub/TargetAngleDeg", Math.toDegrees(angleRad));
    Logger.recordOutput("TrackHub/DistanceMeters", distMeters);
    Logger.recordOutput("TrackHub/ShooterRPM", rpm);
    Logger.recordOutput("TrackHub/HoodAngleDeg", hoodDeg);
    Logger.recordOutput("TrackHub/ShooterSurfaceSpeedMPS", surfaceMps);
    Logger.recordOutput("TrackHub/ReadyToShoot", turret.isReadyToShoot());
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopAll();
  }

  /** This command never finishes on its own; interrupt it to stop tracking. */
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Linear interpolation over a {@link TreeMap} of (distance -> value) pairs.
   *
   * <p>If {@code key} is below the lowest key the first value is returned; above the highest key
   * the last value is returned.
   *
   * @param key lookup distance in metres
   * @param dataMap ordered map of distance -> setpoint (RPM or degrees)
   * @return interpolated setpoint value
   */
  private static double interpolate(double key, TreeMap<Double, Double> dataMap) {
    if (dataMap.isEmpty()) return 0.0;

    Double lo = dataMap.floorKey(key);
    Double hi = dataMap.ceilingKey(key);

    if (lo == null) return dataMap.get(hi);
    if (hi == null) return dataMap.get(lo);
    if (lo.equals(hi)) return dataMap.get(lo);

    double y1 = dataMap.get(lo);
    double y2 = dataMap.get(hi);
    double fraction = (key - lo) / (hi - lo);
    return y1 + (y2 - y1) * fraction;
  }
}
