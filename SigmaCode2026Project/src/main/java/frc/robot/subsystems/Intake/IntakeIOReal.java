package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private TalonFX roller;

  public IntakeIOReal() {
    roller = new TalonFX(IntakeConstants.rollerID, new CANBus("PhoenixBus"));

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.rollerCurrentLimit;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    roller.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRollerVoltage = roller.getMotorVoltage().getValueAsDouble();
    inputs.intakeRollerCurrent = roller.getStatorCurrent().getValueAsDouble();
    inputs.intakeRollerDisconnected = !roller.isConnected();
  }

  @Override
  public void setIntakeRollerSpeed(double speed) {
    roller.set(speed);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    roller.setVoltage(voltage);
  }
}
