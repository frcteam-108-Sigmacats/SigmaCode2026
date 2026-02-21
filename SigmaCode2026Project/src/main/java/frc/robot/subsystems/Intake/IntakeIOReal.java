package frc.robot.subsystems.Intake;

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
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal implements IntakeIO {
  private SparkMax actuation;
  private SparkFlex roller;
  private SparkClosedLoopController actuaterController;
  private RelativeEncoder actuaterIntEnc;
  private SparkMaxConfig actuaterConfig = new SparkMaxConfig();
  private SparkFlexConfig rollerConfig = new SparkFlexConfig();

  public IntakeIOReal() {
    actuation = new SparkMax(IntakeConstants.actuaterID, MotorType.kBrushless);
    roller = new SparkFlex(IntakeConstants.rollerID, MotorType.kBrushless);

    actuaterConfig.smartCurrentLimit(IntakeConstants.actuaterCurrentLimit);
    actuaterConfig.idleMode(IdleMode.kCoast);

    actuaterConfig.closedLoop.pid(
        IntakeConstants.actuaterP, IntakeConstants.actuaterI, IntakeConstants.actuaterD);
    actuaterConfig.closedLoop.positionWrappingEnabled(true);
    actuaterConfig.closedLoop.outputRange(0, 360);
    actuaterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    actuaterConfig.encoder.positionConversionFactor(IntakeConstants.actuaterPositionConversion);
    actuaterConfig.encoder.inverted(IntakeConstants.actucaterEncoderInverted);

    rollerConfig.smartCurrentLimit(IntakeConstants.rollerCurrentLimit);
    rollerConfig.idleMode(IdleMode.kCoast);

    actuation.configure(
        actuaterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    actuaterController = actuation.getClosedLoopController();
    actuaterIntEnc = actuation.getEncoder();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeActuaterVoltage = actuation.getAppliedOutput();
    inputs.intakeActuaterCurrent = actuation.getOutputCurrent();
    inputs.intakeActuaterDisconnected = actuation.getLastError() == REVLibError.kCANDisconnected;
    inputs.intakeActuaterAngle = Rotation2d.fromDegrees(actuaterIntEnc.getPosition());

    inputs.intakeRollerVoltage = roller.getAppliedOutput();
    inputs.intakeRollerCurrent = roller.getOutputCurrent();
    inputs.intakeRollerDisconnected = roller.getLastError() == REVLibError.kCANDisconnected;
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
