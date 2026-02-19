package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

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
import com.revrobotics.spark.SparkBase;
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
 * Full competition-robot implementation (IOReal) for the Turret subsystem.
 *
 * <p>Differences from {@link TurretIOMix}:
 * <ul>
 *   <li>Turret uses the <em>absolute</em> encoder (through-bore) for homing so position
 *       survives power cycles without re-zeroing.
 *   <li>Hood idle mode set to {@code kBrake} at all times for safety.
 *   <li>Shooter wheels hold {@code kCoast} only while disabled (standard competition config).
 * </ul>
 *
 * <p>Motors:
 * <ul>
 *   <li>Turret rotation – Neo Vortex via {@link SparkFlex}
 *   <li>Shooter wheels – 2× Kraken X60 via {@link TalonFX}
 *   <li>Hood – Neo 550 via {@link SparkMax}
 * </ul>
 */
public class TurretIOReal implements TurretIO {

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final SparkFlex turretMotor;
  private final TalonFX shooterLeft;
  private final TalonFX shooterRight;
  private final SparkMax hoodMotor;

  // ── Encoders / Controllers ────────────────────────────────────────────────
  /**
   * Absolute encoder attached to the turret output shaft (through-bore).
   * Reports position in radians after applying {@code positionConversionFactor}.
   */
  private final AbsoluteEncoder turretAbsoluteEncoder;
  private final SparkClosedLoopController turretController;

  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodController;

  // ── Connection debouncers ─────────────────────────────────────────────────
  private final Debouncer turretDebounce = new Debouncer(0.5);
  private final Debouncer hoodDebounce = new Debouncer(0.5);

  public TurretIOReal() {

    // ── Turret Rotation (SparkFlex / Vortex) ─────────────────────────────
    turretMotor = new SparkFlex(turretRotationCanId, MotorType.kBrushless);
    turretAbsoluteEncoder = turretMotor.getAbsoluteEncoder();
    turretController = turretMotor.getClosedLoopController();

    var turretCfg = new SparkFlexConfig();
    turretCfg
        .inverted(turretInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turretCurrentLimitAmps)
        .voltageCompensation(12.0);
    // Absolute encoder covers a full turret revolution; convert to radians.
    turretCfg
        .absoluteEncoder
        .positionConversionFactor(2.0 * Math.PI) // raw [0,1] → rad
        .velocityConversionFactor((2.0 * Math.PI) / 60.0)
        .inverted(false)
        .averageDepth(2);
    turretCfg
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(turretKp, 0.0, turretKd);
    turretCfg
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turretMotor,
        5,
        () ->
            turretMotor.configure(
                turretCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // ── Shooter Wheels (TalonFX / Kraken X60) ────────────────────────────
    shooterLeft = new TalonFX(shooterWheelLeftCanId, canBusName);
    shooterRight = new TalonFX(shooterWheelRightCanId, canBusName);

    var shooterCfg = new TalonFXConfiguration();
    shooterCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterCfg.CurrentLimits.StatorCurrentLimit = shooterCurrentLimitAmps;
    shooterCfg.Feedback.SensorToMechanismRatio = shooterWheelGearRatio;
    shooterCfg.Slot0.kP = shooterKp;
    shooterCfg.Slot0.kI = 0.0;
    shooterCfg.Slot0.kD = shooterKd;
    shooterCfg.Slot0.kS = shooterKs;
    shooterCfg.Slot0.kV = shooterKv;

    shooterLeft.getConfigurator().apply(shooterCfg);

    shooterCfg.MotorOutput.Inverted =
        shooterRightInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    shooterRight.getConfigurator().apply(shooterCfg);

    // ── Hood (SparkMax / Neo 550) ─────────────────────────────────────────
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
    tryUntilOk(
        hoodMotor,
        5,
        () ->
            hoodMotor.configure(
                hoodCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    hoodMotor.getEncoder().setPosition(0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // ── Turret ────────────────────────────────────────────────────────────
    sparkStickyFault = false;
    ifOk(
        turretMotor,
        turretAbsoluteEncoder::getPosition,
        v ->
            // Wrap raw [0, 2π] reading to [-π, π]
            inputs.turretPositionRad = MathUtil.angleModulus(v));
    ifOk(
        turretMotor,
        turretAbsoluteEncoder::getVelocity,
        v -> inputs.turretVelocityRadPerSec = v);
    ifOk(
        turretMotor,
        new java.util.function.DoubleSupplier[] {
          turretMotor::getAppliedOutput, turretMotor::getBusVoltage
        },
        vals -> inputs.turretAppliedVolts = vals[0] * vals[1]);
    ifOk(turretMotor, turretMotor::getOutputCurrent, v -> inputs.turretCurrentAmps = v);
    inputs.turretConnected =
        turretDebounce.calculate(turretMotor.getLastError() == REVLibError.kOk);

    // ── Shooter Wheels ────────────────────────────────────────────────────
    inputs.shooterLeftConnected = shooterLeft.isConnected();
    inputs.shooterRightConnected = shooterRight.isConnected();

    inputs.shooterLeftVelocityRadPerSec = shooterLeft.getVelocity().getValueAsDouble();
    inputs.shooterRightVelocityRadPerSec = shooterRight.getVelocity().getValueAsDouble();
    inputs.shooterLeftAppliedVolts = shooterLeft.getMotorVoltage().getValueAsDouble();
    inputs.shooterRightAppliedVolts = shooterRight.getMotorVoltage().getValueAsDouble();
    inputs.shooterLeftCurrentAmps = shooterLeft.getStatorCurrent().getValueAsDouble();
    inputs.shooterRightCurrentAmps = shooterRight.getStatorCurrent().getValueAsDouble();

    // ── Hood ──────────────────────────────────────────────────────────────
    sparkStickyFault = false;
    ifOk(hoodMotor, hoodEncoder::getPosition, v -> inputs.hoodPositionDeg = v);
    ifOk(hoodMotor, hoodEncoder::getVelocity, v -> inputs.hoodVelocityDegPerSec = v);
    ifOk(
        hoodMotor,
        new java.util.function.DoubleSupplier[] {
          hoodMotor::getAppliedOutput, hoodMotor::getBusVoltage
        },
        vals -> inputs.hoodAppliedVolts = vals[0] * vals[1]);
    ifOk(hoodMotor, hoodMotor::getOutputCurrent, v -> inputs.hoodCurrentAmps = v);
    inputs.hoodConnected = hoodDebounce.calculate(hoodMotor.getLastError() == REVLibError.kOk);
  }

  // ── Turret Commands ───────────────────────────────────────────────────────

  @Override
  public void setTurretOpenLoop(double outputVolts) {
    turretMotor.setVoltage(outputVolts);
  }

  @Override
  public void setTurretPosition(double angleRad) {
    double wrapped = MathUtil.angleModulus(angleRad);
    turretController.setReference(wrapped, ControlType.kPosition);
  }

  // ── Shooter Wheel Commands ────────────────────────────────────────────────

  @Override
  public void setShooterOpenLoop(double outputVolts) {
    shooterLeft.setVoltage(outputVolts);
    shooterRight.setVoltage(outputVolts);
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    double ffVolts = shooterKs * Math.signum(velocityRadPerSec) + shooterKv * velocityRadPerSec;
    shooterLeft.setControl(
        shooterVelocityRequest
            .withSlot(0)
            .withVelocity(velocityRadPerSec)
            .withFeedForward(ffVolts));
    shooterRight.setControl(
        shooterVelocityRequest
            .withSlot(0)
            .withVelocity(velocityRadPerSec)
            .withFeedForward(ffVolts));
  }

  // ── Hood Commands ─────────────────────────────────────────────────────────

  @Override
  public void setHoodOpenLoop(double outputVolts) {
    hoodMotor.setVoltage(outputVolts);
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    double clamped = MathUtil.clamp(angleDeg, hoodMinDeg, hoodMaxDeg);
    hoodController.setReference(clamped, ControlType.kPosition);
  }
}
