package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class TurretConstants {

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
  public static final double turretGearRatio = 100.0 / 1.0;

  public static final double turretEncoderPositionFactor =
      (2.0 * Math.PI) / turretGearRatio; // rad per motor rotation
  public static final double turretEncoderVelocityFactor =
      turretEncoderPositionFactor / 60.0; // rad/s per RPM

  /** Soft limits in radians (±180 °). */
  public static final double turretMinAngleRad = -Math.PI;

  public static final double turretMaxAngleRad = Math.PI;

  // Closed-loop gains (real)
  public static final double turretKp = 1.5;
  public static final double turretKd = 0.0;

  // Closed-loop gains (sim)
  public static final double turretSimKp = 8.0;
  public static final double turretSimKd = 0.0;

  // ── Shooter Wheels (Kraken / TalonFX) ────────────────────────────────────
  public static final double shooterWheelGearRatio = 1.0; // direct drive
  public static final int shooterCurrentLimitAmps = 60;
  public static final boolean shooterRightInverted = true;

  // Velocity PID / FF (Slot 0 on TalonFX)
  public static final double shooterKp = 0.05;
  public static final double shooterKd = 0.0;
  public static final double shooterKs = 0.05; // V
  public static final double shooterKv = 0.12; // V·s/rad

  // Sim
  public static final double shooterSimKp = 0.5;
  public static final double shooterSimKd = 0.0;

  // ── Hood (Neo 550 / SparkMax) ─────────────────────────────────────────────
  public static final boolean hoodInverted = false;
  public static final int hoodCurrentLimitAmps = 20;
  /** Gear ratio: motor rotations per 1 hood degree. */
  public static final double hoodGearRatio = 50.0 / 1.0;

  public static final double hoodEncoderPositionFactor =
      360.0 / hoodGearRatio; // degrees per motor rotation
  public static final double hoodEncoderVelocityFactor =
      hoodEncoderPositionFactor / 60.0; // deg/s per RPM

  /** Hood travel limits in degrees. */
  public static final double hoodMinDeg = 0.0;

  public static final double hoodMaxDeg = 90.0;

  // Closed-loop gains (real)
  public static final double hoodKp = 0.1;
  public static final double hoodKd = 0.0;

  // Closed-loop gains (sim)
  public static final double hoodSimKp = 5.0;
  public static final double hoodSimKd = 0.0;

  // ── DCMotor models used for simulation ───────────────────────────────────
  public static final DCMotor turretMotorModel = DCMotor.getNeoVortex(1);
  public static final DCMotor shooterMotorModel = DCMotor.getKrakenX60(2);
  public static final DCMotor hoodMotorModel = DCMotor.getNeo550(1);

  // Moments of inertia for sim (kg·m²)
  public static final double turretMOI = 0.5;
  public static final double shooterWheelMOI = 0.025;
  public static final double hoodMOI = 0.01;

  // Shooter wheel radius (m) – used for surface-speed calcs
  public static final double shooterWheelRadiusMeters = Units.inchesToMeters(3.0);
}
