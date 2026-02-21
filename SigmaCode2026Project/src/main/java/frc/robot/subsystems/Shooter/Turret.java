package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.TurretIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // Velocity tolerance for "at speed" check (rad/s)
  private static final double SHOOTER_VELOCITY_TOLERANCE_RAD_PER_SEC = 10.0;
  // Angle tolerance for "on target" check (rad)
  private static final double TURRET_ANGLE_TOLERANCE_RAD = Math.toRadians(1.5);
  // Hood position tolerance (deg)
  private static final double HOOD_TOLERANCE_DEG = 1.5;

  private double desiredShooterVelocityRadPerSec = 0.0;
  private double desiredTurretAngleRad = 0.0;
  private double desiredHoodAngleDeg = 0.0;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Derived outputs for dashboard / logging
    Logger.recordOutput("Turret/OnTarget", isOnTarget());
    Logger.recordOutput("Turret/ShooterAtSpeed", isShooterAtSpeed());
    Logger.recordOutput("Turret/HoodAtPosition", isHoodAtPosition());
    Logger.recordOutput("Turret/ReadyToShoot", isReadyToShoot());
    Logger.recordOutput(
        "Turret/ShooterSurfaceMPS",
        inputs.shooterLeftVelocityRadPerSec * TurretConstants.shooterWheelRadiusMeters);
  }

  // ── Turret Rotation API ───────────────────────────────────────────────────

  /** Command the turret to a target angle. Wraps to [-π, π]. */
  public void setTurretAngle(Rotation2d angle) {
    desiredTurretAngleRad = angle.getRadians();
    io.setTurretPosition(desiredTurretAngleRad);
  }

  /** Run the turret rotation motor open-loop (volts). */
  public void setTurretOpenLoop(double volts) {
    io.setTurretOpenLoop(volts);
  }

  /** Stop the turret rotation motor. */
  public void stopTurret() {
    io.setTurretOpenLoop(0.0);
  }

  /** Current turret angle as a {@link Rotation2d}. */
  public Rotation2d getTurretAngle() {
    return new Rotation2d(inputs.turretPositionRad);
  }

  /** Whether the turret is within {@value TURRET_ANGLE_TOLERANCE_RAD} rad of its target. */
  public boolean isOnTarget() {
    return Math.abs(inputs.turretPositionRad - desiredTurretAngleRad) < TURRET_ANGLE_TOLERANCE_RAD;
  }

  // ── Shooter Flywheel API ──────────────────────────────────────────────────

  /**
   * Spin both shooter wheels to the requested surface speed in meters per second.
   *
   * @param metersPerSec desired surface speed (positive = outward)
   */
  public void setShooterSpeed(double metersPerSec) {
    desiredShooterVelocityRadPerSec = metersPerSec / TurretConstants.shooterWheelRadiusMeters;
    io.setShooterVelocity(desiredShooterVelocityRadPerSec);
  }

  /** Run shooter wheels open-loop (volts). */
  public void setShooterOpenLoop(double volts) {
    io.setShooterOpenLoop(volts);
  }

  /** Coast both wheels to a stop. */
  public void stopShooter() {
    io.setShooterOpenLoop(0.0);
  }

  /** Average shooter wheel velocity in rad/s. */
  public double getShooterVelocityRadPerSec() {
    return (inputs.shooterLeftVelocityRadPerSec + inputs.shooterRightVelocityRadPerSec) / 2.0;
  }

  /** Whether both wheels are within tolerance of the current setpoint. */
  public boolean isShooterAtSpeed() {
    if (desiredShooterVelocityRadPerSec == 0.0) return false;
    return Math.abs(inputs.shooterLeftVelocityRadPerSec - desiredShooterVelocityRadPerSec)
            < SHOOTER_VELOCITY_TOLERANCE_RAD_PER_SEC
        && Math.abs(inputs.shooterRightVelocityRadPerSec - desiredShooterVelocityRadPerSec)
            < SHOOTER_VELOCITY_TOLERANCE_RAD_PER_SEC;
  }

  // ── Hood API ──────────────────────────────────────────────────────────────

  /**
   * Command the hood to a target elevation angle in degrees.
   *
   * @param angleDeg angle in [0°, 90°]
   */
  public void setHoodAngle(double angleDeg) {
    desiredHoodAngleDeg = angleDeg;
    io.setHoodPosition(angleDeg);
  }

  /** Run hood motor open-loop (volts). */
  public void setHoodOpenLoop(double volts) {
    io.setHoodOpenLoop(volts);
  }

  /** Stop the hood motor. */
  public void stopHood() {
    io.setHoodOpenLoop(0.0);
  }

  /** Current hood elevation in degrees. */
  public double getHoodAngleDeg() {
    return inputs.hoodPositionDeg;
  }

  /** Whether the hood is within {@value HOOD_TOLERANCE_DEG}° of its target. */
  public boolean isHoodAtPosition() {
    return Math.abs(inputs.hoodPositionDeg - desiredHoodAngleDeg) < HOOD_TOLERANCE_DEG;
  }

  // ── Composite ─────────────────────────────────────────────────────────────

  /**
   * Returns {@code true} when the turret is on target, shooter wheels are at speed, and the hood is
   * at the desired elevation – i.e. the robot is ready to fire.
   */
  public boolean isReadyToShoot() {
    return isOnTarget() && isShooterAtSpeed() && isHoodAtPosition();
  }

  /** Stop all motors immediately. */
  public void stopAll() {
    stopTurret();
    stopShooter();
    stopHood();
  }
}
