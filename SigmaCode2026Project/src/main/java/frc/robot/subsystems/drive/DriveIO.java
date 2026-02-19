package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the swerve drive subsystem. This interface defines the inputs and outputs for
 * the drive hardware.
 */
public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    // Gyro
    public double gyroYawDegrees = 0.0;
    public double gyroPitchDegrees = 0.0;
    public double gyroRollDegrees = 0.0;
    public double gyroYawVelocityDegPerSec = 0.0;
    public boolean gyroConnected = false;

    // Module states (4 modules: FL, FR, BL, BR)
    public double[] moduleSteerPositionsDeg = new double[4];
    public double[] moduleSteerVelocitiesDegPerSec = new double[4];
    public double[] moduleDrivePositionsMeters = new double[4];
    public double[] moduleDriveVelocitiesMetersPerSec = new double[4];
    public double[] moduleSteerCurrentAmps = new double[4];
    public double[] moduleDriveCurrentAmps = new double[4];
    public double[] moduleSteerTempCelsius = new double[4];
    public double[] moduleDriveTempCelsius = new double[4];

    // Odometry
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryYawPositions = new double[] {};
    public double[][] odometryDrivePositionsMeters = new double[][] {new double[4]};
    public Rotation2d[] odometrySteerPositions =
        new Rotation2d[] {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setDriveVoltage(double[] voltages) {}

  /** Run closed loop at the specified velocity. */
  public default void setDriveVelocity(double[] velocitiesMetersPerSec) {}

  /** Set the steer angle for each module. */
  public default void setSteerPosition(Rotation2d[] positions) {}

  /** Enable or disable brake mode for drive motors. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode for steer motors. */
  public default void setSteerBrakeMode(boolean enable) {}

  /** Stop all drive and steer motors. */
  public default void stop() {}
}
