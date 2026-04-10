package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.pigeonCanId;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId, new CANBus("PhoenixBus"));
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<Angle> pitch =
      pigeon.getRoll(); // Gyro is backwards so roll is the pitch
  private final StatusSignal<Angle> roll = pigeon.getPitch(); // Gyro is backwards so pitch is roll

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      pigeon.getConfigurator().setYaw(0.0);
    } else {
      pigeon.getConfigurator().setYaw(180.0);
    }
    yaw.setUpdateFrequency(250); // set later idk
    yawVelocity.setUpdateFrequency(250.0);
    roll.setUpdateFrequency(250);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = SparkXPhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkXPhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, roll).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
    inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(pitch.getValueAsDouble());
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(roll.getValueAsDouble());
  }

  @Override
  public double getRate() {
    return pigeon.getAngularVelocityZDevice().getValueAsDouble();
  }

  @Override
  public void resetGyro() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      pigeon.setYaw(0);
    } else {
      pigeon.setYaw(180);
    }
  }
}
