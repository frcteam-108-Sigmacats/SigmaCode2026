package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;

/**
 * Real hardware implementation for the shooter subsystem. Uses CTRE TalonFX (Kraken X60) for
 * flywheel motors, REV SparkFlex (Vortex) for turret, and REV SparkMax (Neo 550) for hood.
 */
public class ShooterIOReal implements ShooterIO {
  private final TalonFX leftShooterMotor;
  private final TalonFX rightShooterMotor;

  private final SparkFlex turretMotor;
  private final SparkMax hoodMotor;

  private final CANcoder turretEncoder;

  private final SparkClosedLoopController turretController;
  private final SparkClosedLoopController hoodController;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public ShooterIOReal() {
    // Initialize flywheel motors (Kraken X60)
    leftShooterMotor = new TalonFX(ShooterConstants.kLeftShooterMotorId);
    rightShooterMotor = new TalonFX(ShooterConstants.kRightShooterMotorId);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kShooterCurrentLimit;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Configure PID for velocity control
    flywheelConfig.Slot0.kP = ShooterConstants.kShooterP;
    flywheelConfig.Slot0.kI = ShooterConstants.kShooterI;
    flywheelConfig.Slot0.kD = ShooterConstants.kShooterD;
    flywheelConfig.Slot0.kV = ShooterConstants.kShooterFF;

    leftShooterMotor.getConfigurator().apply(flywheelConfig);

    flywheelConfig.MotorOutput.Inverted =
        com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
    rightShooterMotor.getConfigurator().apply(flywheelConfig);

    // Initialize turret motor (Neo Vortex)
    turretMotor = new SparkFlex(ShooterConstants.kTurretMotorId, MotorType.kBrushless);

    SparkFlexConfig turretConfig = new SparkFlexConfig();
    turretConfig.smartCurrentLimit(ShooterConstants.kTurretCurrentLimit);
    turretConfig.idleMode(SparkFlexConfig.IdleMode.kBrake);
    turretConfig.inverted(ShooterConstants.kTurretInverted);

    turretConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kAbsoluteEncoder)
        .pid(ShooterConstants.kTurretP, ShooterConstants.kTurretI, ShooterConstants.kTurretD);

    turretMotor.configure(
        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretController = turretMotor.getClosedLoopController();

    // Initialize turret absolute encoder
    turretEncoder = new CANcoder(ShooterConstants.kTurretEncoderId);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.kTurretOffset.getRotations();
    turretEncoder.getConfigurator().apply(encoderConfig);

    // Initialize hood motor (Neo 550)
    hoodMotor = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.smartCurrentLimit(ShooterConstants.kHoodCurrentLimit);
    hoodConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    hoodConfig.inverted(ShooterConstants.kHoodInverted);

    hoodConfig
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kHoodP, ShooterConstants.kHoodI, ShooterConstants.kHoodD);

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodController = hoodMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Flywheel motors
    inputs.leftShooterVelocityRPM = leftShooterMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.rightShooterVelocityRPM = rightShooterMotor.getVelocity().getValueAsDouble() * 60.0;

    inputs.leftShooterAppliedVolts = leftShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightShooterAppliedVolts = rightShooterMotor.getMotorVoltage().getValueAsDouble();

    inputs.leftShooterCurrentAmps = leftShooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightShooterCurrentAmps = rightShooterMotor.getSupplyCurrent().getValueAsDouble();

    inputs.leftShooterTempCelsius = leftShooterMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightShooterTempCelsius = rightShooterMotor.getDeviceTemp().getValueAsDouble();

    // Turret
    inputs.turretPositionDeg = turretEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    inputs.turretVelocityDegPerSec = turretEncoder.getVelocity().getValueAsDouble() * 360.0;
    inputs.turretAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
    inputs.turretCurrentAmps = turretMotor.getOutputCurrent();
    inputs.turretTempCelsius = turretMotor.getMotorTemperature();

    // Hood
    inputs.hoodPositionDeg = hoodMotor.getEncoder().getPosition();
    inputs.hoodVelocityDegPerSec = hoodMotor.getEncoder().getVelocity() / 60.0;
    inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
    inputs.hoodCurrentAmps = hoodMotor.getOutputCurrent();
    inputs.hoodTempCelsius = hoodMotor.getMotorTemperature();
  }

  @Override
  public void setFlywheelVelocity(double leftRPM, double rightRPM) {
    leftShooterMotor.setControl(velocityRequest.withVelocity(leftRPM / 60.0));
    rightShooterMotor.setControl(velocityRequest.withVelocity(rightRPM / 60.0));
  }

  @Override
  public void setTurretPosition(double degrees) {
    turretController.setReference(degrees, SparkFlex.ControlType.kPosition);
  }

  @Override
  public void setHoodPosition(double degrees) {
    hoodController.setReference(degrees, SparkMax.ControlType.kPosition);
  }

  @Override
  public void stop() {
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
    turretMotor.stopMotor();
    hoodMotor.stopMotor();
  }
}
