package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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
 * Mixed-vendor real-hardware implementation of {@link TurretIO}.
 *
 * <ul>
 *   <li>Turret rotation – Neo Vortex via {@link SparkFlex} with relative encoder
 *   <li>Shooter wheels  – 2× Kraken X60 via {@link TalonFX} (Phoenix 6)
 *   <li>Hood            – Neo 550 via {@link SparkMax}
 * </ul>
 *
 * <p>Sensor reads are performed directly without {@code ifOk} wrappers; connection
 * state is tracked via a sticky-fault debouncer on the last REV error code and the
 * Phoenix {@code isConnected()} method.
 */
public class TurretIOMix implements TurretIO {

  // ── Hardware ──────────────────────────────────────────────────────────────
  private final SparkFlex turretMotor;
  private final TalonFX   shooterLeft;
  private final TalonFX   shooterRight;
  private final SparkMax  hoodMotor;

  // ── Encoders / controllers ────────────────────────────────────────────────
  private final RelativeEncoder          turretEncoder;
  private final SparkClosedLoopController turretController;
  private final VelocityVoltage          shooterVelocityReq = new VelocityVoltage(0);
  private final RelativeEncoder          hoodEncoder;
  private final SparkClosedLoopController hoodController;

  // ── Connection debounce ───────────────────────────────────────────────────
  private final Debouncer turretDebounce = new Debouncer(0.5);
  private final Debouncer hoodDebounce   = new Debouncer(0.5);

  public TurretIOMix() {

    // ── Turret (SparkFlex / Vortex) ───────────────────────────────────────
    turretMotor     = new SparkFlex(turretRotationCanId, MotorType.kBrushless);
    turretEncoder   = turretMotor.getEncoder();
    turretController = turretMotor.getClosedLoopController();

    var turretCfg = new SparkFlexConfig();
    turretCfg
        .inverted(turretInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turretCurrentLimitAmps)
        .voltageCompensation(12.0);
    turretCfg.encoder
        .positionConversionFactor(turretEncoderPositionFactor)
        .velocityConversionFactor(turretEncoderVelocityFactor);
    turretCfg.closedLoop
        .pid(turretKp, 0.0, turretKd);
    turretCfg.softLimit
        .forwardSoftLimit((float) turretMaxAngleRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit((float) turretMinAngleRad)
        .reverseSoftLimitEnabled(true);

    turretMotor.configure(turretCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretMotor.getEncoder().setPosition(0.0);

    // ── Shooter wheels (TalonFX / Kraken X60) ────────────────────────────
    shooterLeft  = new TalonFX(shooterWheelLeftCanId,  canBusName);
    shooterRight = new TalonFX(shooterWheelRightCanId, canBusName);

    var shooterCfg = new TalonFXConfiguration();
    shooterCfg.MotorOutput.NeutralMode              = NeutralModeValue.Coast;
    shooterCfg.CurrentLimits.StatorCurrentLimit     = shooterCurrentLimitAmps;
    shooterCfg.Feedback.SensorToMechanismRatio      = shooterWheelGearRatio;
    shooterCfg.Slot0.kP = shooterKp;
    shooterCfg.Slot0.kI = 0.0;
    shooterCfg.Slot0.kD = shooterKd;
    shooterCfg.Slot0.kS = shooterKs;
    shooterCfg.Slot0.kV = shooterKv;

    // Left wheel – natural direction
    shooterCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterLeft.getConfigurator().apply(shooterCfg);

    // Right wheel – mirrored so both wheels eject in the same direction
    shooterCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterRight.getConfigurator().apply(shooterCfg);

    // ── Hood (SparkMax / Neo 550) ──────────────────────────────────────────
    hoodMotor    = new SparkMax(hoodCanId, MotorType.kBrushless);
    hoodEncoder  = hoodMotor.getEncoder();
    hoodController = hoodMotor.getClosedLoopController();

    var hoodCfg = new SparkMaxConfig();
    hoodCfg
        .inverted(hoodInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(hoodCurrentLimitAmps)
        .voltageCompensation(12.0);
    hoodCfg.encoder
        .positionConversionFactor(hoodEncoderPositionFactor)
        .velocityConversionFactor(hoodEncoderVelocityFactor);
    hoodCfg.closedLoop
        .pid(hoodKp, 0.0, hoodKd);
    hoodCfg.softLimit
        .forwardSoftLimit((float) hoodMaxDeg)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit((float) hoodMinDeg)
        .reverseSoftLimitEnabled(true);

    hoodMotor.configure(hoodCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodMotor.getEncoder().setPosition(0.0);
  }

  // ── updateInputs ──────────────────────────────────────────────────────────

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // Turret
    inputs.turretPositionRad       = turretEncoder.getPosition();
    inputs.turretVelocityRadPerSec = turretEncoder.getVelocity();
    inputs.turretAppliedVolts      = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
    inputs.turretCurrentAmps       = turretMotor.getOutputCurrent();
    inputs.turretConnected         = turretDebounce.calculate(turretMotor.getLastError() == REVLibError.kOk);

    // Shooter wheels
    inputs.shooterLeftConnected          = shooterLeft.isConnected();
    inputs.shooterRightConnected         = shooterRight.isConnected();
    inputs.shooterLeftVelocityRadPerSec  = shooterLeft.getVelocity().getValueAsDouble();
    inputs.shooterRightVelocityRadPerSec = shooterRight.getVelocity().getValueAsDouble();
    inputs.shooterLeftAppliedVolts       = shooterLeft.getMotorVoltage().getValueAsDouble();
    inputs.shooterRightAppliedVolts      = shooterRight.getMotorVoltage().getValueAsDouble();
    inputs.shooterLeftCurrentAmps        = shooterLeft.getStatorCurrent().getValueAsDouble();
    inputs.shooterRightCurrentAmps       = shooterRight.getStatorCurrent().getValueAsDouble();

    // Hood
    inputs.hoodPositionDeg       = hoodEncoder.getPosition();
    inputs.hoodVelocityDegPerSec = hoodEncoder.getVelocity();
    inputs.hoodAppliedVolts      = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
    inputs.hoodCurrentAmps       = hoodMotor.getOutputCurrent();
    inputs.hoodConnected         = hoodDebounce.calculate(hoodMotor.getLastError() == REVLibError.kOk);
  }

  // ── Turret rotation ───────────────────────────────────────────────────────

  @Override
  public void setTurretOpenLoop(double outputVolts) {
    turretMotor.setVoltage(outputVolts);
  }

  @Override
  public void setTurretPosition(double angleRad) {
    turretController.setReference(angleRad, ControlType.kPosition);
  }

  // ── Shooter wheels ────────────────────────────────────────────────────────

  @Override
  public void setShooterOpenLoop(double outputVolts) {
    shooterLeft.setVoltage(outputVolts);
    shooterRight.setVoltage(outputVolts);
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    double ff = shooterKs * Math.signum(velocityRadPerSec) + shooterKv * velocityRadPerSec;
    shooterLeft.setControl(
        shooterVelocityReq.withSlot(0).withVelocity(velocityRadPerSec).withFeedForward(ff));
    shooterRight.setControl(
        shooterVelocityReq.withSlot(0).withVelocity(velocityRadPerSec).withFeedForward(ff));
  }

  // ── Hood ──────────────────────────────────────────────────────────────────

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
