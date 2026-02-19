package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.DriveConstants;

import java.util.Queue;

/**
 * Real hardware implementation for the swerve drive subsystem.
 * Uses CTRE TalonFX (Kraken X60) for drive motors, REV SparkMax (Neo 550) for steering,
 * CTRE CANcoders for absolute position, and Pigeon2 for IMU.
 */
public class DriveIOReal implements DriveIO {
  // Drive motors (Kraken X60)
  private final TalonFX[] driveMotors = new TalonFX[4];
  
  // Steer motors (Neo 550 via SparkMax)
  private final SparkMax[] steerMotors = new SparkMax[4];
  private final SparkClosedLoopController[] steerControllers = new SparkClosedLoopController[4];
  
  // Absolute encoders (CANcoder)
  private final CANcoder[] steerEncoders = new CANcoder[4];
  
  // IMU
  private final Pigeon2 pigeon;

  // Status signals for odometry
  private final Queue<Double> timestampQueue;
  private final StatusSignal<Angle> yawPosition;
  private final StatusSignal<Double>[] drivePositions = new StatusSignal[4];
  private final StatusSignal<Double>[] steerPositions = new StatusSignal[4];

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public DriveIOReal() {
    // Initialize drive motors (Kraken X60)
    int[] driveMotorIds = {
      DriveConstants.kFrontLeftDriveMotorId,
      DriveConstants.kFrontRightDriveMotorId,
      DriveConstants.kBackLeftDriveMotorId,
      DriveConstants.kBackRightDriveMotorId
    };

    for (int i = 0; i < 4; i++) {
      driveMotors[i] = new TalonFX(driveMotorIds[i]);
      
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.CurrentLimits.SupplyCurrentLimit = DriveConstants.kDriveCurrentLimit;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      
      // Configure PID for velocity control
      config.Slot0.kP = DriveConstants.kDriveP;
      config.Slot0.kI = DriveConstants.kDriveI;
      config.Slot0.kD = DriveConstants.kDriveD;
      
      driveMotors[i].getConfigurator().apply(config);
      
      // Get status signals for odometry
      drivePositions[i] = driveMotors[i].getPosition();
    }

    // Initialize steer motors (Neo 550 via SparkMax)
    int[] steerMotorIds = {
      DriveConstants.kFrontLeftSteerMotorId,
      DriveConstants.kFrontRightSteerMotorId,
      DriveConstants.kBackLeftSteerMotorId,
      DriveConstants.kBackRightSteerMotorId
    };

    for (int i = 0; i < 4; i++) {
      steerMotors[i] = new SparkMax(steerMotorIds[i], MotorType.kBrushless);
      
      SparkMaxConfig config = new SparkMaxConfig();
      config.smartCurrentLimit(DriveConstants.kSteerCurrentLimit);
      config.idleMode(SparkMaxConfig.IdleMode.kBrake);
      
      // Configure closed loop control
      config.closedLoop
          .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
          .pid(DriveConstants.kSteerP, DriveConstants.kSteerI, DriveConstants.kSteerD);
      
      steerMotors[i].configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      steerControllers[i] = steerMotors[i].getClosedLoopController();
    }

    // Initialize absolute encoders (CANcoder)
    int[] encoderIds = {
      DriveConstants.kFrontLeftEncoderId,
      DriveConstants.kFrontRightEncoderId,
      DriveConstants.kBackLeftEncoderId,
      DriveConstants.kBackRightEncoderId
    };

    Rotation2d[] offsets = {
      DriveConstants.kFrontLeftEncoderOffset,
      DriveConstants.kFrontRightEncoderOffset,
      DriveConstants.kBackLeftEncoderOffset,
      DriveConstants.kBackRightEncoderOffset
    };

    for (int i = 0; i < 4; i++) {
      steerEncoders[i] = new CANcoder(encoderIds[i]);
      
      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.MagnetOffset = offsets[i].getRotations();
      
      steerEncoders[i].getConfigurator().apply(config);
      
      // Get status signals for odometry
      steerPositions[i] = steerEncoders[i].getAbsolutePosition();
    }

    // Initialize Pigeon2
    pigeon = new Pigeon2(DriveConstants.kPigeonId);
    Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
    pigeon.getConfigurator().apply(pigeonConfig);
    pigeon.reset();
    
    yawPosition = pigeon.getYaw();

    // Create timestamp queue for odometry
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Update gyro
    inputs.gyroYawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
        pigeon.getYaw(), pigeon.getAngularVelocityZWorld());
    inputs.gyroPitchDegrees = pigeon.getPitch().getValueAsDouble();
    inputs.gyroRollDegrees = pigeon.getRoll().getValueAsDouble();
    inputs.gyroYawVelocityDegPerSec = pigeon.getAngularVelocityZWorld().getValueAsDouble();
    inputs.gyroConnected = BaseStatusSignal.isAllGood(pigeon.getYaw());

    // Update module states
    for (int i = 0; i < 4; i++) {
      // Steer
      inputs.moduleSteerPositionsDeg[i] = 
          steerEncoders[i].getAbsolutePosition().getValueAsDouble() * 360.0;
      inputs.moduleSteerVelocitiesDegPerSec[i] = 
          steerEncoders[i].getVelocity().getValueAsDouble() * 360.0;
      inputs.moduleSteerCurrentAmps[i] = steerMotors[i].getOutputCurrent();
      inputs.moduleSteerTempCelsius[i] = steerMotors[i].getMotorTemperature();

      // Drive
      double driveRotations = driveMotors[i].getPosition().getValueAsDouble();
      inputs.moduleDrivePositionsMeters[i] = 
          (driveRotations / DriveConstants.kDriveGearRatio) * DriveConstants.kWheelCircumference;
      
      double driveRPM = driveMotors[i].getVelocity().getValueAsDouble() * 60.0;
      inputs.moduleDriveVelocitiesMetersPerSec[i] = 
          (driveRPM / 60.0 / DriveConstants.kDriveGearRatio) * DriveConstants.kWheelCircumference;
      
      inputs.moduleDriveCurrentAmps[i] = driveMotors[i].getSupplyCurrent().getValueAsDouble();
      inputs.moduleDriveTempCelsius[i] = driveMotors[i].getDeviceTemp().getValueAsDouble();
    }

    // Update odometry data
    inputs.odometryTimestamps = 
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = 
        PhoenixOdometryThread.getInstance().getSignalQueue(yawPosition).stream()
            .mapToDouble((Double value) -> value).toArray();
    
    inputs.odometryDrivePositionsMeters = new double[drivePositions.length][];
    for (int i = 0; i < drivePositions.length; i++) {
      inputs.odometryDrivePositionsMeters[i] = 
          PhoenixOdometryThread.getInstance().getSignalQueue(drivePositions[i]).stream()
              .mapToDouble((Double value) -> 
                  (value / DriveConstants.kDriveGearRatio) * DriveConstants.kWheelCircumference)
              .toArray();
    }
    
    inputs.odometrySteerPositions = new Rotation2d[steerPositions.length];
    for (int i = 0; i < steerPositions.length; i++) {
      inputs.odometrySteerPositions[i] = 
          Rotation2d.fromRotations(
              PhoenixOdometryThread.getInstance().getSignalQueue(steerPositions[i]).stream()
                  .mapToDouble((Double value) -> value).toArray()[0]);
    }

    timestampQueue.clear();
  }

  @Override
  public void setDriveVoltage(double[] voltages) {
    for (int i = 0; i < 4; i++) {
      driveMotors[i].setControl(voltageRequest.withOutput(voltages[i]));
    }
  }

  @Override
  public void setDriveVelocity(double[] velocitiesMetersPerSec) {
    for (int i = 0; i < 4; i++) {
      double rotationsPerSec = velocitiesMetersPerSec[i] / DriveConstants.kWheelCircumference 
          * DriveConstants.kDriveGearRatio;
      driveMotors[i].setControl(velocityRequest.withVelocity(rotationsPerSec));
    }
  }

  @Override
  public void setSteerPosition(Rotation2d[] positions) {
    for (int i = 0; i < 4; i++) {
      steerControllers[i].setReference(
          positions[i].getRotations(), 
          SparkMax.ControlType.kPosition);
    }
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    NeutralModeValue mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    for (TalonFX motor : driveMotors) {
      motor.setNeutralMode(mode);
    }
  }

  @Override
  public void setSteerBrakeMode(boolean enable) {
    SparkMaxConfig.IdleMode mode = enable ? 
        SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast;
    for (SparkMax motor : steerMotors) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.idleMode(mode);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  @Override
  public void stop() {
    for (TalonFX motor : driveMotors) {
      motor.stopMotor();
    }
    for (SparkMax motor : steerMotors) {
      motor.stopMotor();
    }
  }
}
