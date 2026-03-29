package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    // Gyro is inverted so the roll is the pitch and pitch is the roll
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d rollPosition = new Rotation2d();
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  @AutoLogOutput
  public default Rotation2d getYaw() {
    return new Rotation2d();
  }

  @AutoLogOutput
  public default Rotation2d getPitch() {
    return new Rotation2d();
  }

  @AutoLogOutput
  public default Rotation2d getRoll() {
    return new Rotation2d();
  }

  public default double getRate() {
    return 0;
  }
}
