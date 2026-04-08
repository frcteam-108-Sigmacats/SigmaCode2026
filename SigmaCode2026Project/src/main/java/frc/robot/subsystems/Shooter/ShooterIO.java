package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Hardware abstraction layer for the Shooter subsystem.
 *
 * <p>Motors on the real robot:
 *
 * <ul>
 *   <li>Turret rotation – 1× Neo Vortex (SparkFlex)
 *   <li>Shooter wheels – 2× Kraken X60 (TalonFX)
 *   <li>Hood – 1× Neo 550 (SparkMax)
 * </ul>
 */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {

    // ── Turret Rotation ──────────────────────────────────────────────────────
    public boolean turretConnected = false;
    /** Current turret angle in radians (0 = forward). */
    public Rotation2d turretPosition = new Rotation2d();

    public double turretVelocityRadPerSec = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;

    // ── Shooter Wheels ───────────────────────────────────────────────────────
    public boolean shooterLeftConnected = false;
    public boolean shooterRightConnected = false;

    public double shooterLeftVelocityRPM = 0.0;
    public double shooterRightVelocityRPM = 0.0;
    public double shooterLeftAppliedVolts = 0.0;
    public double shooterRightAppliedVolts = 0.0;
    public double shooterLeftCurrentAmps = 0.0;
    public double shooterRightCurrentAmps = 0.0;

    // ── Hood ─────────────────────────────────────────────────────────────────
    public boolean hoodConnected = false;
    /** Hood position in degrees read from the internal (relative) encoder (0 = lowest / flat). */
    public double hoodPositionDeg = 0.0;

    /** Hood position in degrees read directly from the absolute encoder. */
    public double hoodAbsPositionDeg = 0.0;

    public double hoodVelocityDegPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  @AutoLogOutput
  /** Push sensor data into the {@link TurretIOInputs} snapshot. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  // ── Turret Rotation Commands ─────────────────────────────────────────────

  /** Run the turret rotation motor at the given open-loop voltage (-12 … +12 V). */
  public default void setTurretOpenLoop(double outputVolts) {}

  /** Command the turret to a target angle (radians) using closed-loop control. */
  public default void setTurretPosition(double angleRad) {}

  // ── Shooter Wheel Commands ───────────────────────────────────────────────

  /** Run both shooter wheels at the given open-loop voltage. */
  public default void setShooterOpenLoop(double outputVolts) {}

  /** Command both shooter wheels to a target surface velocity (rad/s) using closed-loop control. */
  public default void setShooterVelocity(double velocityRotPerMin) {}

  // ── Hood Commands ────────────────────────────────────────────────────────

  /** Run the hood motor at the given open-loop voltage. */
  public default void setHoodOpenLoop(double outputVolts) {}

  /** Command the hood to a target angle (degrees) using closed-loop control. */
  public default void setHoodPosition(double angleDeg) {}
}
