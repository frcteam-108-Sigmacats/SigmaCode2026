package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ModuleIOMix implements ModuleIO {
  private final Rotation2d zeroRotation;

  private final TalonFX driveMotor;
  private final SparkBase turnMotor;

  private final AbsoluteEncoder turnEncoder;

  private final VelocityVoltage velocity = new VelocityVoltage(0);
  private final SparkClosedLoopController turnController;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOMix(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> new Rotation2d();
        };
    driveMotor =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            "*");
    turnMotor =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnEncoder = turnMotor.getAbsoluteEncoder();
    turnController = turnMotor.getClosedLoopController();

    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.driveMotorCurrentLimit;
    driveConfig.Feedback.SensorToMechanismRatio = driveEncoderPositionFactor;
    driveConfig.Slot0.kP = driveKp;
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = driveKd;
    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setPosition(0);

    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnMotor,
        5,
        () ->
            turnMotor.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    timestampQueue = SparkXPhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkXPhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition());
    turnPositionQueue =
        SparkXPhoenixOdometryThread.getInstance()
            .registerSignal(turnMotor, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    sparkStickyFault = false;
    inputs.drivePositionRad = driveMotor.getPosition().getValueAsDouble();
    inputs.driveVelocityRadPerSec = driveMotor.getVelocity().getValueAsDouble();
    inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();
    inputs.driveConnected = driveMotor.isConnected();

    sparkStickyFault = false;
    ifOk(
        turnMotor,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnMotor, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnMotor,
        new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnMotor.getLastError() == REVLibError.kOk;

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);

    int n =
        Math.min(
            inputs.odometryTimestamps.length,
            Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length));

    if (n < inputs.odometryTimestamps.length) {
      inputs.odometryTimestamps = Arrays.copyOf(inputs.odometryTimestamps, n);
    }
    if (n < inputs.odometryDrivePositionsRad.length) {
      inputs.odometryDrivePositionsRad = Arrays.copyOf(inputs.odometryDrivePositionsRad, n);
    }
    if (n < inputs.odometryTurnPositions.length) {
      inputs.odometryTurnPositions = Arrays.copyOf(inputs.odometryTurnPositions, n);
    }

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;

    driveMotor.setControl(
        velocity.withSlot(0).withFeedForward(ffVolts).withVelocity(velocityRadPerSec));
  }

  @Override
  public void setTurnPos(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
