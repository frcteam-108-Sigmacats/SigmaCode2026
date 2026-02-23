package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.TreeMap;

public class ShooterConstants {

  // ── CAN IDs ──────────────────────────────────────────────────────────────
  /** SparkFlex (Vortex) driving the turret rotation ring. */
  public static final int turretRotationCanId = 20;

  /** TalonFX (Kraken X60) for the left shooter wheel. */
  public static final int shooterWheelLeftCanId = 21;

  /** TalonFX (Kraken X60) for the right shooter wheel. */
  public static final int shooterWheelRightCanId = 22;

  /** SparkMax (Neo 550) for the shooter hood angle. */
  public static final int hoodCanId = 23;

  // ── CAN Bus ──────────────────────────────────────────────────────────────
  /** CAN bus name for TalonFX devices. Use "*" or "rio" as appropriate. */
  public static final String canBusName = "*";

  // ── Turret Rotation (Vortex / SparkFlex) ─────────────────────────────────
  public static final boolean turretInverted = false;
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

  // Closed-loop gains (real hardware)
  public static final double turretKp = 1.5;
  public static final double turretKd = 0.0;

  // Closed-loop gains (sim)
  public static final double turretSimKp = 8.0;
  public static final double turretSimKd = 0.0;

  // ── Shooter Wheels (Kraken / TalonFX) ────────────────────────────────────
  public static final double shooterWheelGearRatio = 1.0; // direct drive
  public static final int shooterCurrentLimitAmps = 40; // change later if too low
  public static final boolean shooterRightInverted = true;

  // Velocity PID / FF (Slot 0 on TalonFX)
  public static final double shooterKp = 0.05;
  public static final double shooterKd = 0.0;
  public static final double shooterKs = 0.05; // V
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

  public static final double hoodMaxDeg = 90.0;

  // Closed-loop gains (real hardware)
  public static final double hoodKp = 0.1;
  public static final double hoodKd = 0.0;

  // Closed-loop gains (sim)
  public static final double hoodSimKp = 5.0;
  public static final double hoodSimKd = 0.0;

  // ── DCMotor models used for simulation ───────────────────────────────────
  public static final DCMotor turretMotorModel = DCMotor.getNeoVortex(1);
  public static final DCMotor shooterMotorModel = DCMotor.getKrakenX60(2);
  public static final DCMotor hoodMotorModel = DCMotor.getNeo550(1);

  // Moments of inertia for sim (kg*m^2)
  public static final double turretMOI = 0.5;
  public static final double shooterWheelMOI = 0.025;
  public static final double hoodMOI = 0.01;

  /** Shooter flywheel radius (m) – 3 in contact wheel. */
  public static final double shooterWheelRadiusMeters = Units.inchesToMeters(3.0);

  /** Shooter inertia wheel radius (m) – 4 in inertia wheels. */
  public static final double shooterInertiaWheelRadiusMeters = Units.inchesToMeters(4.0);

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

    /**
     * Hood elevation in degrees as a function of distance (metres). Interpolated linearly between
     * known points.
     */
    public static final TreeMap<Double, Double> shooterHoodAngleMap = new TreeMap<>();

    static {
      // Distance (m) -> RPM
      shooterRPMMap.put(1.5, 2800.0);
      shooterRPMMap.put(2.0, 3000.0);
      shooterRPMMap.put(3.0, 3400.0);
      shooterRPMMap.put(4.0, 3900.0);
      shooterRPMMap.put(5.0, 4500.0);
      shooterRPMMap.put(6.0, 5200.0);

      // Distance (m) -> hood angle (deg)
      shooterHoodAngleMap.put(1.5, 60.0);
      shooterHoodAngleMap.put(2.0, 55.0);
      shooterHoodAngleMap.put(3.0, 48.0);
      shooterHoodAngleMap.put(4.0, 42.0);
      shooterHoodAngleMap.put(5.0, 36.0);
      shooterHoodAngleMap.put(6.0, 30.0);
    }
  }
}
