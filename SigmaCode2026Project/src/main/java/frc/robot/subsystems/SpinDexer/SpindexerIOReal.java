package frc.robot.subsystems.SpinDexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SpinDexerIOReal implements SpinDexerIO {
  private SparkMax spinDexer;

  private SparkFlex kicker1;
  private SparkFlex kicker2;

  private SparkMaxConfig configSpinDexer = new SparkMaxConfig();
  private SparkFlexConfig configSparkFlex = new SparkFlexConfig();

  private CANrange countingSensor;

  private CANrangeConfiguration canRangeConfiguration = new CANrangeConfiguration();

  public SpinDexerIOReal() {
    spinDexer = new SparkMax(SpinDexerConstants.spinDexerID, MotorType.kBrushless);
    kicker1 = new SparkFlex(SpinDexerConstants.kicker1ID, MotorType.kBrushless);
    kicker2 = new SparkFlex(SpinDexerConstants.kicker2ID, MotorType.kBrushless);

    countingSensor = new CANrange(SpinDexerConstants.canRangeID, new CANBus("PhoenixBus"));

    // canRangeConfiguration.ProximityParams.ProximityThreshold = 0.0;

    countingSensor.getConfigurator().apply(canRangeConfiguration);
    configSpinDexer.smartCurrentLimit(SpinDexerConstants.spinDexerCurrentLimit);
    configSpinDexer.idleMode(IdleMode.kCoast);
    configSparkFlex.smartCurrentLimit(40);
    configSparkFlex.idleMode(IdleMode.kBrake);

    spinDexer.configure(
        configSpinDexer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker1.configure(
        configSparkFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker2.configure(
        configSparkFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SpinDexerIOInputs inputs) {
    inputs.spinDexerDisconnected = spinDexer.getLastError() == REVLibError.kCANDisconnected;
    inputs.spinDexerKickerMotorVoltage = spinDexer.getAppliedOutput();
    inputs.spinDexerMotorCurrent = spinDexer.getOutputCurrent();
    inputs.spinDexerKickerDisconnected = kicker1.getLastError() == REVLibError.kCANDisconnected;
    inputs.spinDexerKickerMotorVoltage = kicker1.getAppliedOutput();
    inputs.spinDexerMotorCurrent = kicker1.getOutputCurrent();

    inputs.detectBall = countingSensor.getIsDetected().getValue();
  }

  @Override
  public void setSpinDexerSpeed(double speed) {
    spinDexer.set(speed);
  }

  @Override
  public void setKickerSpeed(double speed) {
    kicker1.set(speed);
    kicker2.set(speed);
  }

  @Override
  public void setSpinDexerVoltage(double volt) {
    spinDexer.setVoltage(volt);
  }

  @Override
  public void setKickerVoltage(double volt) {
    kicker1.setVoltage(volt);
    kicker2.setVoltage(volt);
  }
}
