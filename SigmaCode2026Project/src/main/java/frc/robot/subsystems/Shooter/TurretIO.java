package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction layer for the Turret subsystem.
 *
 * <p>Motors on the real robot:
 * <ul>
 *   <li>Turret rotation – 1× Neo Vortex (SparkFlex)
 *   <li>Shooter wheels  – 2× Kraken X60 (TalonFX)
 *   <li>Hood            – 1× Neo 550 (SparkMax)
 * </ul>
 *
 * <p>All three concrete implementations (IOSim, IOMix, IOReal) override every
 * method so the public surface is identical across all variants.
 */
public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {

    // ── Turret Rotation ──────────────────────────────────────────────────
    public boolean turretConnected          = false;
    /** Current turret yaw in radians, wrapped to [-π, π]. */
    public double  turretPositionRad        = 0.0;
    public double  turretVelocityRadPerSec  = 0.0;
    public double  turretAppliedVolts       = 0.0;
    public double  turretCurrentAmps        = 0.0;

    // ── Shooter Wheels ───────────────────────────────────────────────────
    public boolean shooterLeftConnected          = false;
    public boolean shooterRightConnected         = false;
    public double  shooterLeftVelocityRadPerSec  = 0.0;
    public double  shooterRightVelocityRadPerSec = 0.0;
    public double  shooterLeftAppliedVolts       = 0.0;
    public double  shooterRightAppliedVolts      = 0.0;
    public double  shooterLeftCurrentAmps        = 0.0;
    public double  shooterRightCurrentAmps       = 0.0;

    // ── Hood ─────────────────────────────────────────────────────────────
    public boolean hoodConnected         = false;
    /** Hood elevation in degrees – 0 = flat, 90 = straight up. */
    public double  hoodPositionDeg       = 0.0;
    public double  hoodVelocityDegPerSec = 0.0;
    public double  hoodAppliedVolts      = 0.0;
    public double  hoodCurrentAmps       = 0.0;
  }

  // ── Periodic update ──────────────────────────────────────────────────────

  /** Refresh all sensor readings into {@code inputs}. Called every loop cycle. */
  public default void updateInputs(TurretIOInputs inputs) {}

  // ── Turret rotation ──────────────────────────────────────────────────────

  /** Drive the turret motor at a fixed voltage (-12 to +12 V). */
  public default void setTurretOpenLoop(double outputVolts) {}

  /** Servo the turret to {@code angleRad} using onboard closed-loop control. */
  public default void setTurretPosition(double angleRad) {}

  // ── Shooter wheels ───────────────────────────────────────────────────────

  /** Drive both shooter wheels at a fixed voltage (-12 to +12 V). */
  public default void setShooterOpenLoop(double outputVolts) {}

  /**
   * Run both shooter wheels at {@code velocityRadPerSec} using onboard
   * closed-loop velocity control with feedforward.
   */
  public default void setShooterVelocity(double velocityRadPerSec) {}

  // ── Hood ─────────────────────────────────────────────────────────────────

  /** Drive the hood motor at a fixed voltage (-12 to +12 V). */
  public default void setHoodOpenLoop(double outputVolts) {}

  /** Servo the hood to {@code angleDeg} using onboard closed-loop control. */
  public default void setHoodPosition(double angleDeg) {}
}
