package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.TreeMap;

public class ShooterConstants {

  // ── CAN IDs ──────────────────────────────────────────────────────────────
  /** SparkFlex (Vortex) driving the turret rotation ring. */
  public static final int turretRotationCanId = 17;

  /** TalonFX (Kraken X60) for the left shooter wheel. */
  public static final int shooterWheelLeftCanId = 15;

  /** TalonFX (Kraken X60) for the right shooter wheel. */
  public static final int shooterWheelRightCanId = 16;

  /** SparkMax (Neo 550) for the shooter hood angle. */
  public static final int hoodCanId = 18;

  // ── CAN Bus ──────────────────────────────────────────────────────────────
  /** CAN bus name for TalonFX devices. Use "*" or "rio" as appropriate. */
  public static final String canBusName = "*";

  // ── Turret Rotation (Vortex / SparkFlex) ─────────────────────────────────
  public static final boolean turretInverted = true;
  public static final int turretCurrentLimitAmps = 40;

  /** Gear ratio: motor rotations per 1 turret rotation. */
  public static final double turretGearRatio = 31.5;

  public static final double turretEncoderPositionFactor =
      (2.0 * Math.PI) / turretGearRatio; // rad per motor rotation
  public static final double turretEncoderVelocityFactor =
      turretEncoderPositionFactor / 60.0; // rad/s per RPM

  /** Soft limits in radians (+/- 180 deg). */
  public static final double turretMinAngleRad = -Math.PI;

  public static final double turretMaxAngleRad = Math.PI;

  public static final double turretForwardLimit = 1.919; // was 2.8;
  public static final double turretReverseLimit = -4.7;

  // Closed-loop gains (real hardware)
  public static final double turretKp = 1.0; // 1.5
  public static final double turretKd = 0.01;

  // Closed-loop gains (sim)
  public static final double turretSimKp = 8.0;
  public static final double turretSimKd = 0.0;

  // ── Shooter Wheels (Kraken / TalonFX) ────────────────────────────────────
  public static final double shooterWheelGearRatio = 1.0; // direct drive
  public static final int shooterCurrentLimitAmps = 40; // change later if too low
  public static final boolean shooterRightInverted = true;

  // Velocity PID / FF (Slot 0 on TalonFX)
  public static final double shooterKp = 0.01;
  public static final double shooterKd = 0.0;
  public static final double shooterKs = 0.01; // V
  public static final double shooterKv = 0.12; // V*s/rad

  // Sim
  public static final double shooterSimKp = 0.5;
  public static final double shooterSimKd = 0.0;

  // ── Hood (Neo 550 / SparkMax) ─────────────────────────────────────────────
  public static final boolean hoodInverted = false;
  public static final int hoodCurrentLimitAmps = 20;

  /** Gear ratio: motor rotations per 1 hood degree. */
  public static final double hoodGearRatio = 66.67;

  public static final double hoodEncoderPositionFactor =
      360.0 / hoodGearRatio; // degrees per motor rotation
  public static final double hoodEncoderVelocityFactor =
      hoodEncoderPositionFactor / 60.0; // deg/s per RPM

  /** Hood travel limits in degrees. */
  public static final double hoodMinDeg = 0.0;

  public static final double hoodMaxDeg = 25.0;

  // Closed-loop gains (real hardware)
  public static final double hoodKp = 0.05;
  public static final double hoodKd = 0.0;

  // Closed-loop gains (sim)
  public static final double hoodSimKp = 5.0;
  public static final double hoodSimKd = 0.0;

  public static final double hoodStartAngle = 12;

  public static final double ballExitVelocityConversion = 0.002626;

  // ── DCMotor models used for simulation ───────────────────────────────────
  public static final DCMotor turretMotorModel = DCMotor.getNeoVortex(1);
  public static final DCMotor shooterMotorModel = DCMotor.getKrakenX60(2);
  public static final DCMotor hoodMotorModel = DCMotor.getNeo550(1);

  // Moments of inertia for sim (kg*m^2)
  public static final double turretMOI = 0.5;
  public static final double shooterWheelMOI = 0.025;
  public static final double hoodMOI = 0.01;

  /** Shooter flywheel radius (m) – 3 in contact wheel. */
  public static final double shooterWheelRadiusMeters = Units.inchesToMeters(1.5);

  /** Shooter inertia wheel radius (m) – 4 in inertia wheels. */
  public static final double shooterInertiaWheelRadiusMeters = Units.inchesToMeters(4.0);

  public static final Pose2d blueHubPose = new Pose2d(4.62, 4.03, new Rotation2d());
  public static final Pose2d blueDepotPose = new Pose2d(2.5, 7, new Rotation2d());
  public static final Pose2d blueStationPose =
      new Pose2d(2.5, 1.06, new Rotation2d()); // Put Station Pose
    
  public static enum ShooterStatus{
    SHOOT,
    PASSING,
    INTAKE
  }

  // ── Shooter lookup tables ─────────────────────────────────────────────────
  /**
   * Distance-to-setpoint lookup tables used by {@link frc.robot.commands.TrackHubCommand}.
   *
   * <p>Key = distance from robot to target in metres.
   *
   * <p>Value = shooter wheel RPM / hood angle in degrees.
   *
   * <p>Add or adjust rows after characterising the shooter on your field.
   */
  public static final class ShooterStates {

    private ShooterStates() {}

    /**
     * Shooter wheel speed in RPM as a function of distance (metres). Interpolated linearly between
     * known points.
     */
    public static final TreeMap<Double, Double> shooterRPMMap = new TreeMap<>();

    static {
      // Distance (m) -> RPM
      shooterRPMMap.put((double) 0, 2500.0);
      shooterRPMMap.put((double) 1.117, 2700.0);
      shooterRPMMap.put((double) 2.488, 2700.0);
      shooterRPMMap.put((double) 3.135, 2850.0);
      shooterRPMMap.put((double) 3.9, 3130.0);
      shooterRPMMap.put((double) 4.4, 3230.0);
      shooterRPMMap.put((double) 5, 3380.0);
      shooterRPMMap.put((double) 5.2, 3450.0);
      shooterRPMMap.put((double) 6.149, 3600.0);
      shooterRPMMap.put((double) 7.63, 3800.0);
    }

    /**
     * Hood elevation in degrees as a function of distance (metres). Interpolated linearly between
     * known points.
     */
    public static final TreeMap<Double, Double> shooterHoodAngleMap = new TreeMap<>();

    static {

      // Distance (m) -> hood angle (deg)
      shooterHoodAngleMap.put((double) 0, 0.0);
      shooterHoodAngleMap.put((double) 1.117, 12.0);
      shooterHoodAngleMap.put((double) 2.488, 18.0);
      shooterHoodAngleMap.put((double) 3.135, 20.0);
      shooterHoodAngleMap.put((double) 3.9, 22.0);
      shooterHoodAngleMap.put((double) 4.4, 24.0);
      shooterHoodAngleMap.put((double) 5, 25.0);
    }
  }
}
