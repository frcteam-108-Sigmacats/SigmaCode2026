// this was writen late at night and is not ready yet
// will not work by its self needs the drive subsystem will be removed later

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.Shooter.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public class TrackHubCommand extends Command {

  private final Turret turret;
  private final Drive drive;

  /**
   * Fraction of robot velocity used for motion compensation. A value of 1.0 means full one-step
   * prediction; tune lower if over-correcting.
   */
  private static final double MOTION_COMP_FACTOR = 0.5;

  /**
   * Shooter wheel diameter in metres – used to convert RPM setpoints from the lookup table into
   * surface speed (m/s) for {@link Turret#setShooterSpeed}.
   */
  private static final double WHEEL_DIAMETER_M = 0.1524; // 6 inch wheels

  public TrackHubCommand(Turret turret, Drive drive) {
    this.turret = turret;
    this.drive = drive;
    addRequirements(turret);
    // Drive is read-only here; do NOT add it as a requirement
  }

  @Override
  public void initialize() {
    // Nothing to initialise – command calculates fresh each cycle
  }

  @Override
  public void execute() {

    Translation2d robotPos = drive.getPose().getTranslation();
    Rotation2d robotYaw = drive.getPose().getRotation();
    Translation2d hubPos = Constants.PoseConstants.blueHubPose;

    Translation2d rawVec = hubPos.minus(robotPos); // vector robot → hub
    double rawDist = rawVec.getNorm(); // straight-line distance (m)

    double rpmRaw = interpolate(rawDist, TurretConstants.ShooterStates.shooterRPMMap);
    double hoodDegRaw = interpolate(rawDist, TurretConstants.ShooterStates.shooterHoodAngleMap);

    // Estimate flight time from wheel surface speed and shooting distance.
    // surfaceSpeed (m/s) = (RPM / 60) * π * diameter
    double surfaceSpeedMps = (rpmRaw / 60.0) * Math.PI * WHEEL_DIAMETER_M;
    // Horizontal component only (adjusted by hood angle)
    double horizontalSpeed = surfaceSpeedMps * Math.cos(Math.toRadians(hoodDegRaw));
    double flightTimeSec = (horizontalSpeed > 0.01) ? (rawDist / horizontalSpeed) : 0.0;

    // Predict where the hub "appears" to be from a moving robot's perspective
    Translation2d robotVel =
        new Translation2d(
            drive.getDriveSpeeds().vxMetersPerSecond * MOTION_COMP_FACTOR,
            drive.getDriveSpeeds().vyMetersPerSecond * MOTION_COMP_FACTOR);

    Translation2d compensatedHub =
        new Translation2d(
            hubPos.getX() - robotVel.getX() * flightTimeSec,
            hubPos.getY() - robotVel.getY() * flightTimeSec);

    Translation2d aimVec = compensatedHub.minus(robotPos);
    double aimDist = aimVec.getNorm();

    double rpmFinal = interpolate(aimDist, ShooterConstants.ShooterStates.shooterRPMMap);
    double hoodDegFinal = interpolate(aimDist, ShooterConstants.ShooterStates.shooterHoodAngleMap);

    // Field-relative bearing to the compensated aim point
    Rotation2d fieldAngle = Rotation2d.fromRadians(Math.atan2(aimVec.getY(), aimVec.getX()));

    // Subtract robot yaw to get the required turret rotation from the robot's
    // forward axis. The turret is mounted with 0 = robot forward.
    Rotation2d turretSetpoint = fieldAngle.minus(robotYaw);

    double surfaceSpeedFinal = (rpmFinal / 60.0) * Math.PI * WHEEL_DIAMETER_M;

    turret.setTurretAngle(turretSetpoint);
    turret.setShooterSpeed(surfaceSpeedFinal);
    turret.setHoodAngle(hoodDegFinal);

    Logger.recordOutput("TrackHub/RawDistanceMeters", rawDist);
    Logger.recordOutput("TrackHub/AimDistanceMeters", aimDist);
    Logger.recordOutput("TrackHub/TurretSetpointDeg", turretSetpoint.getDegrees());
    Logger.recordOutput("TrackHub/ShooterRPM", rpmFinal);
    Logger.recordOutput("TrackHub/HoodAngleDeg", hoodDegFinal);
    Logger.recordOutput("TrackHub/ShooterSurfaceSpeedMPS", surfaceSpeedFinal);
    Logger.recordOutput("TrackHub/ReadyToShoot", turret.isReadyToShoot());
    Logger.recordOutput("TrackHub/CompensatedHub", compensatedHub);
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
   * Linear interpolation over a {@link TreeMap} of (distance → value) pairs.
   *
   * <p>If {@code key} is below the lowest key the first value is returned; above the highest key
   * the last value is returned.
   *
   * @param key lookup distance in metres
   * @param dataMap ordered map of distance → setpoint (RPM or degrees)
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
