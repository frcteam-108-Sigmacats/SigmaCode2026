package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  // Instantiates the Speed Controller
  private TalonFX roller;
  private TalonFX roller2;

  public IntakeIOReal() {
    // Assigns the Speed Controller to the specified CAN ID and CAN Bus
    roller = new TalonFX(IntakeConstants.rollerID, new CANBus("PhoenixBus"));
    roller2 = new TalonFX(IntakeConstants.roller2ID, new CANBus("PhoenixBus"));
    // Instantiates and adds the configurations for the Speed Controller
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    // Enables and sets max current limit for the speed controller
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.rollerCurrentLimit;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Sets the speed controller neutral mode
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Applies the speed controller configurations
    roller.getConfigurator().apply(rollerConfig);
    roller2.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRollerVoltage = roller.getMotorVoltage().getValueAsDouble();
    inputs.intakeRollerCurrent = roller.getStatorCurrent().getValueAsDouble();
    inputs.intakeRollerDisconnected = !roller.isConnected();

    inputs.intakeRoller2Voltage = roller2.getMotorVoltage().getValueAsDouble();
    inputs.intakeRoller2Current = roller2.getStatorCurrent().getValueAsDouble();
    inputs.intakeRoller2Disconnected = !roller2.isConnected();
  }

  @Override
  public void setIntakeRollerSpeed(double speed) {
    roller.set(speed);
    roller2.set(-speed);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    roller.setVoltage(voltage);
    roller2.setVoltage(-voltage);
  }
}
