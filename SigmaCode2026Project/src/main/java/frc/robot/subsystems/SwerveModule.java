package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
  private final int moduleNumber;

  private final TalonFX driveMotorfr;
  private final TalonFX driveMotorfl;
  private final TalonFX driveMotorbl;
  private final TalonFX driveMotorbr;

  private final SparkMax turnMotorfr;
  private final SparkMax turnMotorfl;
  private final SparkMax turnMotorbr;
  private final SparkMax turnMotorbl;

  private SparkAbsoluteEncoder absoluteEncoder;

  private final VelocityVoltage driveRequest = new VelocityVoltage(0);

  // Module constants
  private static final double DRIVE_GEAR_RATIO = 6.75; // Adjust for your gear ratio
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0; // Adjust for your gear ratio
  private static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

  public SwerveModule(int moduleNumber, int driveMotorID, int turnMotorID) {
    this.moduleNumber = moduleNumber;

    // Initialize drive motor (Kraken X60)
    driveMotorfr = new TalonFX(1);
    driveMotorfl = new TalonFX(3);
    driveMotorbr = new TalonFX(5);
    driveMotorbl = new TalonFX(7);
    configureDriveMotor();

    // Initialize turn motor (NEO 550)
    turnMotorfr = new SparkMax(2, MotorType.kBrushless);
    turnMotorfl = new SparkMax(4, MotorType.kBrushless);
    turnMotorbr = new SparkMax(6, MotorType.kBrushless);
    turnMotorbl = new SparkMax(8, MotorType.kBrushless);
    configureSteerMotor();

    // Get absolute encoder
    absoluteEncoder = turnMotorfr.getAbsoluteEncoder();
    absoluteEncoder = turnMotorfl.getAbsoluteEncoder();
    absoluteEncoder = turnMotorbl.getAbsoluteEncoder();
    absoluteEncoder = turnMotorbr.getAbsoluteEncoder();
  }

  private void configureDriveMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Velocity PID
    config.Slot0.kP = 0.6;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.12;

    driveMotorfr.getConfigurator().apply(config);
    driveMotorfr.setPosition(0);

    driveMotorfl.getConfigurator().apply(config);
    driveMotorfl.setPosition(0);

    driveMotorbl.getConfigurator().apply(config);
    driveMotorbl.setPosition(0);

    driveMotorbr.getConfigurator().apply(config);
    driveMotorbr.setPosition(0);
  }

  private void configureSteerMotor() {
    SparkMaxConfig config = new SparkMaxConfig();

    // Motor configuration
    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    config.smartCurrentLimit(30);

    // Absolute encoder configuration
    config
        .absoluteEncoder
        .positionConversionFactor(360.0) // Degrees
        .velocityConversionFactor(360.0 / 60.0); // Degrees per second

    // Closed loop configuration
    config
        .closedLoop
        .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kAbsoluteEncoder)
        .pid(0.5, 0.0, 0.0) // P, I, D
        .outputRange(-1.0, 1.0);

    // Apply configuration
    turnMotorfr.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotorfl.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotorbl.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotorbr.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the state to avoid spinning more than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getSteerAngle());

    // Set drive velocity
    double velocityRPS = state.speedMetersPerSecond / WHEEL_CIRCUMFERENCE;
    driveMotorfr.setControl(driveRequest.withVelocity(velocityRPS));
    driveMotorfl.setControl(driveRequest.withVelocity(velocityRPS));
    driveMotorbl.setControl(driveRequest.withVelocity(velocityRPS));
    driveMotorbr.setControl(driveRequest.withVelocity(velocityRPS));

    // Set turn position
    turnMotorfr
        .getClosedLoopController()
        .setReference(state.angle.getDegrees(), ControlType.kPosition);

    turnMotorbl
        .getClosedLoopController()
        .setReference(state.angle.getDegrees(), ControlType.kPosition);

    turnMotorfl
        .getClosedLoopController()
        .setReference(state.angle.getDegrees(), ControlType.kPosition);

    turnMotorbr
        .getClosedLoopController()
        .setReference(state.angle.getDegrees(), ControlType.kPosition);
  }
  // sets wheel circumference
  public SwerveModuleState getState() {
    double velocity = driveMotorbl.getVelocity().getValueAsDouble() * 0.5;
    return new SwerveModuleState(velocity, getSteerAngle());
  }
  // sets weele posion
  public SwerveModulePosition getPosition() {
    double distance = driveMotorbl.getPosition().getValueAsDouble() * 0.5;
    return new SwerveModulePosition(distance, getSteerAngle());
  }

  private Rotation2d getSteerAngle() {
    return Rotation2d.fromDegrees(absoluteEncoder.getPosition());
  }

  public void stop() {
    driveMotorbl.stopMotor();

    turnMotorfr.stopMotor();
    turnMotorfl.stopMotor();
    turnMotorbl.stopMotor();
    turnMotorbr.stopMotor();
  }
}
