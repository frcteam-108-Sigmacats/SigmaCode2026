package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal implements IntakeIO {
  // Instantiating the Motors for the Mechanism
  private SparkMax actuation;
  private TalonFX roller;
  // Instantiating the ClosedLoop Controller for the Actuation
  private SparkClosedLoopController actuaterController;
  // Instantiating the Actuator's Internal Encoder
  private RelativeEncoder actuaterIntEnc;
  // Creating the configuration objects for the motors
  private SparkMaxConfig actuaterConfig = new SparkMaxConfig();
  public IntakeIOReal() {
    // Assigning the Motors to their respective CAN IDs
    actuation = new SparkMax(IntakeConstants.actuaterID, MotorType.kBrushless);
    roller = new TalonFX(IntakeConstants.rollerID);

    // Configuring the actuator motor's Current Limit and Idle Mode
    actuaterConfig.smartCurrentLimit(IntakeConstants.actuaterCurrentLimit);
    actuaterConfig.idleMode(IdleMode.kCoast);

    // Configuring the Actuator Motor's ClosedLoop Controller Values regarding PID, FeedBackSensor,
    // and PositionWrapping
    actuaterConfig.closedLoop.pid(
        IntakeConstants.actuaterP, IntakeConstants.actuaterI, IntakeConstants.actuaterD);
    actuaterConfig.closedLoop.positionWrappingEnabled(true);
    actuaterConfig.closedLoop.outputRange(0, 360);
    actuaterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Configuring the Actuator Encoder's Position and deciding if it should be inverted or not
    actuaterConfig.encoder.positionConversionFactor(IntakeConstants.actuaterPositionConversion);
    // actuaterConfig.encoder.inverted(IntakeConstants.actucaterEncoderInverted);

    // Applying the Actuator Motor Configurations
    actuation.configure(
        actuaterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configuring the TalonFX roller motor
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.rollerCurrentLimit;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    roller.getConfigurator().apply(rollerConfig);

    // Assigning the ClosedLoop Controller and Encoder to the Actuator Motor's ClosedLoopController
    // and Internal Encoder
    actuaterController = actuation.getClosedLoopController();
    actuaterIntEnc = actuation.getEncoder();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Updating all data that will be logged into AdvantageScope
    inputs.intakeActuaterVoltage = actuation.getAppliedOutput();
    inputs.intakeActuaterCurrent = actuation.getOutputCurrent();
    inputs.intakeActuaterDisconnected = actuation.getLastError() == REVLibError.kCANDisconnected;
    inputs.intakeActuaterAngle = Rotation2d.fromDegrees(actuaterIntEnc.getPosition());

    inputs.intakeRollerVoltage = roller.getMotorVoltage().getValueAsDouble();
    inputs.intakeRollerCurrent = roller.getStatorCurrent().getValueAsDouble();
    inputs.intakeRollerDisconnected = !roller.isConnected();
  }

  @Override
  public void setIntakeActuaterSpeed(double speed) {
    actuation.set(speed);
  }

  @Override
  public void setIntakeActuaterPosition(double position) {
    actuaterController.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public void setIntakeRollerSpeed(double speed) {
    roller.set(speed);
  }

  @Override
  public void setActuaterVoltage(double voltage) {
    actuation.setVoltage(voltage);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    roller.setVoltage(voltage);
  }
}
