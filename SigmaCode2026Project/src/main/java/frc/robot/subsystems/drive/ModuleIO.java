package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {}
  /**
   * Sets the drive motors with a voltage input
   *
   * @param output units in Voltage -12V to +12V
   */
  public default void setDriveOpenLoop(double output) {}
  /**
   * Sets the turn motors with a voltage input
   *
   * @param output units in Voltage -12V to +12V
   */
  public default void setTurnOpenLoop(double output) {}
  /**
   * Sets the drive to a set velocity
   *
   * @param velocityRadPerSec Velocity in RadPerSec
   */
  public default void setDriveVelocity(double velocityRadPerSec) {}
  /**
   * Sets the turn position with a Rotation2d
   *
   * @param rotation2d angle of the turn motor
   */
  public default void setTurnPos(Rotation2d rotation2d) {}
}
