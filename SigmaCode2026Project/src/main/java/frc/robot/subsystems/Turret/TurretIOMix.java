package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

/**
 * Mixed hardware implementation (IOMix) for the Turret subsystem.
 *
 * <ul>
 *   <li>Turret rotation – Neo Vortex via {@link SparkFlex} (REVLib)
 *   <li>Shooter wheels – 2× Kraken X60 via {@link TalonFX} (Phoenix 6)
 *   <li>Hood – Neo 550 via {@link SparkMax} (REVLib)
 * </ul>
 */
public class TurretIOMix implements TurretIO {

  // ── Hardware ─────────────────────────────────────────────────────────────
  private final SparkFlex turretMotor;
  private final TalonFX shooterLeft;
  private final TalonFX shooterRight;
  private final SparkMax hoodMotor;

  // ── Encoders / Controllers ────────────────────────────────────────────────
  private final RelativeEncoder turretEncoder;
  private final SparkClosedLoopController turretController;
  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);
  private final RelativeEncoder hoodEncoder;
  private final SparkClosedLoopController hoodController;

  // ── Connection debouncers ─────────────────────────────────────────────────
  private final Debouncer turretDebounce = new Debouncer(0.5);
  private final Debouncer hoodDebounce = new Debouncer(0.5);

  public TurretIOMix() {

    // ── Turret Rotation (SparkFlex / Vortex) ─────────────────────────────
    turretMotor = new SparkFlex(turretRotationCanId, MotorType.kBrushless);
    turretEncoder = turretMotor.getEncoder();
    turretController = turretMotor.getClosedLoopController();

    var turretCfg = new SparkFlexConfig();
    turretCfg
        .inverted(turretInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turretCurrentLimitAmps)
        .voltageCompensation(12.0);
    turretCfg
        .encoder
        .positionConversionFactor(turretEncoderPositionFactor)
        .velocityConversionFactor(turretEncoderVelocityFactor);
    turretCfg
        .closedLoop
        .pid(turretKp, 0.0, turretKd)
        .outputRange(turretMinAngleRad, turretMaxAngleRad);
    turretCfg
        .softLimit
        .forwardSoftLimit((float) turretMaxAngleRad)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit((float) turretMinAngleRad)
        .reverseSoftLimitEnabled(true);
    tryUntilOk(
        turretMotor,
        5,
        () ->
            turretMotor.configure(
                turretCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    turretMotor.getEncoder().setPosition(0.0);

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

    // Left wheel – default direction
    shooterLeft.getConfigurator().apply(shooterCfg);

    // Right wheel – inverted so both wheels push in the same direction
    shooterCfg.MotorOutput.Inverted =
        shooterRightInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
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
    ifOk(turretMotor, turretEncoder::getPosition, v -> inputs.turretPositionRad = v);
    ifOk(turretMotor, turretEncoder::getVelocity, v -> inputs.turretVelocityRadPerSec = v);
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
    turretController.setReference(angleRad, ControlType.kPosition);
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
        shooterVelocityRequest.withSlot(0).withVelocity(velocityRadPerSec).withFeedForward(ffVolts));
    shooterRight.setControl(
        shooterVelocityRequest.withSlot(0).withVelocity(velocityRadPerSec).withFeedForward(ffVolts));
  }

  // ── Hood Commands ─────────────────────────────────────────────────────────

  @Override
  public void setHoodOpenLoop(double outputVolts) {
    hoodMotor.setVoltage(outputVolts);
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    hoodController.setReference(angleDeg, ControlType.kPosition);
  }
}
