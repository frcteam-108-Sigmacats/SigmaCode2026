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
import org.littletonrobotics.junction.Logger;

public class SpinDexerIOReal implements SpinDexerIO {
  // Instantiating all Speed Controllers with Config variables
  private SparkMax spinDexer;

  private SparkFlex kicker1;
  private SparkFlex kicker2;

  private SparkMaxConfig configSpinDexer = new SparkMaxConfig();
  private SparkFlexConfig configSparkFlex = new SparkFlexConfig();

  // Instantiating Ball Counting Sensor and creating a Config variable to make the configurations
  private CANrange countingSensor;

  private CANrangeConfiguration canRangeConfiguration = new CANrangeConfiguration();

  public SpinDexerIOReal() {
    // Assigning all Speed Controllers to their respective CAN IDs
    spinDexer = new SparkMax(SpinDexerConstants.spinDexerID, MotorType.kBrushless);
    kicker1 = new SparkFlex(SpinDexerConstants.kicker1ID, MotorType.kBrushless);
    kicker2 = new SparkFlex(SpinDexerConstants.kicker2ID, MotorType.kBrushless);

    // Assigning the Ball Counter to it's respective CAN ID and CAN Bus
    countingSensor = new CANrange(SpinDexerConstants.canRangeID, new CANBus("PhoenixBus"));

    // Configuring the Kickers and SpinDexer Speed Controllers with their Current Limit and Idle
    // Mode
    configSpinDexer.smartCurrentLimit(SpinDexerConstants.spinDexerCurrentLimit);
    configSpinDexer.idleMode(IdleMode.kCoast);
    configSparkFlex.smartCurrentLimit(40);
    configSparkFlex.idleMode(IdleMode.kBrake);

    // Configuring the CANRang Detection distance and signal strength needed to validate that it
    // detects something
    canRangeConfiguration.ProximityParams.ProximityThreshold = 0.2;
    canRangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = 4800;

    // Applies the configuration of the CANRange to the Ball Counter Sensor
    countingSensor.getConfigurator().apply(canRangeConfiguration);
    // Tells the Ball Counter to set the Update Frequency of when it detects something to be
    // 250HZ/250 times per second
    countingSensor.getIsDetected().setUpdateFrequency(250.0);

    // Applies the configurations for the speed controllers
    spinDexer.configure(
        configSpinDexer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker1.configure(
        configSparkFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker2.configure(
        configSparkFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SpinDexerIOInputs inputs) {
    // Assigns the Inputs variables to the values we are looking for inside each speed controller
    inputs.spinDexerDisconnected = spinDexer.getLastError() == REVLibError.kCANDisconnected;
    inputs.spinDexerMotorVoltage = spinDexer.getAppliedOutput();
    inputs.spinDexerMotorCurrent = spinDexer.getOutputCurrent();
    inputs.spinDexerKickerDisconnected = kicker1.getLastError() == REVLibError.kCANDisconnected;
    inputs.spinDexerKickerMotorVoltage = kicker1.getAppliedOutput();
    inputs.spinDexerMotorCurrent = kicker1.getOutputCurrent();

    inputs.detectBall = countingSensor.getIsDetected().getValue();

    // Logging the signal strength that the Ball Counter is reading currently
    Logger.recordOutput(
        "Strength of Proximity", countingSensor.getSignalStrength().getValueAsDouble());
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
