package frc.robot.subsystems.Shooter;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;

/**
 * Competition / full-robot implementation of {@link ShooterIO}.
 *
 * <p>Structure intentionally mirrors {@link ShooterIOSim}: each axis tracks a closed-loop flag and
 * a stored setpoint. The actual control output is delegated to onboard hardware controllers rather
 * than WPILib {@code PIDController}s, but the field layout and {@code updateInputs} shape are
 * identical so the two implementations are easy to compare side-by-side.
 *
 * <ul>
 *   <li>Turret rotation – Neo Vortex via {@link SparkFlex} with absolute through-bore encoder
 *   <li>Shooter wheels – 2x Kraken X60 via {@link TalonFX} (Phoenix 6)
 *   <li>Hood – Neo 550 via {@link SparkMax}
 * </ul>
 */
public class ShooterIOReal implements ShooterIO {

  // ── Hardware ──────────────────────────────────────────────────────────────
  private final SparkFlex turretMotor;
  private final TalonFX shooterLeft;
  private final TalonFX shooterRight;
  private final SparkMax hoodMotor;

  // ── Encoders / controllers ────────────────────────────────────────────────
  /** Absolute encoder on the turret output shaft (through-bore). Radians after conversion. */
  private final AbsoluteEncoder turretAbsEncoder;

  /** Internal relative encoder on the turret motor shaft. Seeded from absolute at startup. */
  private final RelativeEncoder turretEncoder;

  private final SparkClosedLoopController turretController;
  private final VelocityVoltage shooterVelocityReq = new VelocityVoltage(0);
  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodController;

  // ── Connection debounce ───────────────────────────────────────────────────
  private final Debouncer turretDebounce = new Debouncer(0.5);
  private final Debouncer hoodDebounce = new Debouncer(0.5);

  // ── Closed-loop state (mirrors TurretIOSim) ───────────────────────────────
  private boolean turretClosedLoop = false;
  private boolean shooterClosedLoop = false;
  private boolean hoodClosedLoop = false;

  private double turretSetpointRad = 0.0;
  private double shooterSetpointRadPerSec = 0.0;
  private double hoodSetpointDeg = 0.0;

  public ShooterIOReal() {

    // ── Turret (SparkFlex / Vortex, absolute encoder) ─────────────────────
    turretMotor = new SparkFlex(turretRotationCanId, MotorType.kBrushless);
    turretAbsEncoder = turretMotor.getAbsoluteEncoder();
    turretEncoder = turretMotor.getEncoder();
    turretController = turretMotor.getClosedLoopController();

    var turretCfg = new SparkFlexConfig();
    turretCfg
        .inverted(turretInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turretCurrentLimitAmps)
        .voltageCompensation(12.0);
    // Relative encoder: motor rotations -> radians at the turret output
    turretCfg
        .encoder
        .positionConversionFactor(turretEncoderPositionFactor)
        .velocityConversionFactor(turretEncoderVelocityFactor);
    // Absolute encoder: raw [0, 1] revolution -> [-pi, pi] radians
    turretCfg
        .absoluteEncoder
        .positionConversionFactor(2.0 * Math.PI)
        .velocityConversionFactor((2.0 * Math.PI) / 60.0)
        .inverted(false)
        .averageDepth(2);
    turretCfg
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        .pid(turretKp, 0.0, turretKd);
    turretCfg
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    turretMotor.configure(
        turretCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ── Shooter wheels (TalonFX / Kraken X60) ────────────────────────────
    shooterLeft = new TalonFX(shooterWheelLeftCanId, new CANBus("PhoenixBus"));
    shooterRight = new TalonFX(shooterWheelRightCanId, new CANBus("PhoenixBus"));

    var shooterCfg = new TalonFXConfiguration();
    shooterCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterCfg.CurrentLimits.StatorCurrentLimit = shooterCurrentLimitAmps;
    shooterCfg.Feedback.SensorToMechanismRatio = shooterWheelGearRatio;
    shooterCfg.Slot0.kP = shooterKp;
    shooterCfg.Slot0.kI = 0.0;
    shooterCfg.Slot0.kD = shooterKd;
    shooterCfg.Slot0.kS = shooterKs;
    shooterCfg.Slot0.kV = shooterKv;

    shooterCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterLeft.getConfigurator().apply(shooterCfg);

    shooterCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterRight.getConfigurator().apply(shooterCfg);

    // ── Hood (SparkMax / Neo 550) ──────────────────────────────────────────
    hoodMotor = new SparkMax(hoodCanId, MotorType.kBrushless);
    hoodEncoder = hoodMotor.getEncoder();
    hoodController = hoodMotor.getClosedLoopController();

    var hoodCfg = new SparkMaxConfig();
    hoodCfg
        .inverted(hoodInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(hoodCurrentLimitAmps)
        .voltageCompensation(12.0);
    hoodCfg
        .encoder
        .positionConversionFactor(hoodEncoderPositionFactor)
        .velocityConversionFactor(hoodEncoderVelocityFactor);
    hoodCfg.closedLoop.pid(hoodKp, 0.0, hoodKd);
    hoodCfg
        .softLimit
        .forwardSoftLimit((float) hoodMaxDeg)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit((float) hoodMinDeg)
        .reverseSoftLimitEnabled(true);
    hoodCfg
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    hoodMotor.configure(hoodCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ── Seed encoders from known references ───────────────────────────────
    // Seed the turret relative encoder from the absolute encoder reading.
    // The absolute encoder reports [0, 2π]; if the position exceeds π (180°), subtract 2π
    // so the relative encoder starts in the same [-π, π] convention used elsewhere.
    double turretAbsPositionRad = turretAbsEncoder.getPosition();
    if (turretAbsPositionRad > Math.PI) {
      turretAbsPositionRad -= 2.0 * Math.PI;
    }
    turretEncoder.setPosition(turretAbsPositionRad);

    // Seed the hood relative encoder to 0 (mechanism assumed to be at rest / home position).
    hoodEncoder.setPosition(0.0);
  }

  // ── updateInputs ──────────────────────────────────────────────────────────

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // Turret – absolute encoder reports [0, 2pi]; wrap to [-pi, pi]
    inputs.turretConnected =
        turretDebounce.calculate(turretMotor.getLastError() == REVLibError.kOk);
    inputs.turretPositionRad = MathUtil.angleModulus(turretAbsEncoder.getPosition());
    inputs.turretVelocityRadPerSec = turretAbsEncoder.getVelocity();
    inputs.turretAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
    inputs.turretCurrentAmps = turretMotor.getOutputCurrent();

    // Shooter wheels
    inputs.shooterLeftConnected = shooterLeft.isConnected();
    inputs.shooterRightConnected = shooterRight.isConnected();
    inputs.shooterLeftVelocityRadPerSec = shooterLeft.getVelocity().getValueAsDouble();
    inputs.shooterRightVelocityRadPerSec = shooterRight.getVelocity().getValueAsDouble();
    inputs.shooterLeftAppliedVolts = shooterLeft.getMotorVoltage().getValueAsDouble();
    inputs.shooterRightAppliedVolts = shooterRight.getMotorVoltage().getValueAsDouble();
    inputs.shooterLeftCurrentAmps = shooterLeft.getStatorCurrent().getValueAsDouble();
    inputs.shooterRightCurrentAmps = shooterRight.getStatorCurrent().getValueAsDouble();

    // Hood
    inputs.hoodConnected = hoodDebounce.calculate(hoodMotor.getLastError() == REVLibError.kOk);
    inputs.hoodPositionDeg = hoodEncoder.getPosition();
    inputs.hoodVelocityDegPerSec = hoodEncoder.getVelocity();
    inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
    inputs.hoodCurrentAmps = hoodMotor.getOutputCurrent();
  }

  // ── Turret rotation ───────────────────────────────────────────────────────

  @Override
  public void setTurretOpenLoop(double outputVolts) {
    turretClosedLoop = false;
    turretMotor.setVoltage(outputVolts);
  }

  @Override
  public void setTurretPosition(double angleRad) {
    turretClosedLoop = true;
    // Clamp to the physical ±180° range – never wrap the setpoint.
    double clampedRad = MathUtil.clamp(angleRad, -Math.PI, Math.PI);
    turretSetpointRad = clampedRad;
    // Absolute encoder reports [0, 2π]. Convert the clamped [-π, π] angle to that space.
    double encoderSetpoint = clampedRad < 0.0 ? clampedRad + 2.0 * Math.PI : clampedRad;
    turretController.setSetpoint(encoderSetpoint, ControlType.kPosition);
  }

  // ── Shooter wheels ────────────────────────────────────────────────────────

  @Override
  public void setShooterOpenLoop(double outputVolts) {
    shooterClosedLoop = false;
    shooterLeft.setVoltage(outputVolts);
    shooterRight.setVoltage(outputVolts);
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    shooterClosedLoop = true;
    shooterSetpointRadPerSec = velocityRadPerSec;
    double ff = shooterKs * Math.signum(velocityRadPerSec) + shooterKv * velocityRadPerSec;
    shooterLeft.setControl(
        shooterVelocityReq.withSlot(0).withVelocity(velocityRadPerSec).withFeedForward(ff));
    shooterRight.setControl(
        shooterVelocityReq.withSlot(0).withVelocity(velocityRadPerSec).withFeedForward(ff));
  }

  // ── Hood ──────────────────────────────────────────────────────────────────

  @Override
  public void setHoodOpenLoop(double outputVolts) {
    hoodClosedLoop = false;
    hoodMotor.setVoltage(outputVolts);
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    hoodClosedLoop = true;
    hoodSetpointDeg = MathUtil.clamp(angleDeg, hoodMinDeg, hoodMaxDeg);
    hoodController.setSetpoint(hoodSetpointDeg, ControlType.kPosition);
  }

  @Override
  public void seedHoodEncoderFromAbsolute() {
    // No absolute encoder on the hood; the relative encoder is seeded to 0 in the constructor.
    // This method is intentionally a no-op for the real robot.
  }
}
