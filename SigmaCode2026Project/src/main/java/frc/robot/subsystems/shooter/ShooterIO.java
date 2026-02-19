package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the shooter subsystem. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double leftShooterVelocityRPM = 0.0;
    public double rightShooterVelocityRPM = 0.0;

    public double leftShooterAppliedVolts = 0.0;
    public double rightShooterAppliedVolts = 0.0;

    public double leftShooterCurrentAmps = 0.0;
    public double rightShooterCurrentAmps = 0.0;

    public double leftShooterTempCelsius = 0.0;
    public double rightShooterTempCelsius = 0.0;

    public double turretPositionDeg = 0.0;
    public double turretVelocityDegPerSec = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;
    public double turretTempCelsius = 0.0;

    public double hoodPositionDeg = 0.0;
    public double hoodVelocityDegPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
    public double hoodTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Set flywheel velocity in RPM. */
  public default void setFlywheelVelocity(double leftRPM, double rightRPM) {}

  /** Set turret position in degrees. */
  public default void setTurretPosition(double degrees) {}

  /** Set hood position in degrees. */
  public default void setHoodPosition(double degrees) {}

  /** Stop all motors. */
  public default void stop() {}
}
