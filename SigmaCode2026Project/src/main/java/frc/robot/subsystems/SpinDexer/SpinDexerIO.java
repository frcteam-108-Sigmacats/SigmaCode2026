package frc.robot.subsystems.SpinDexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpinDexerIO {
  @AutoLog
  public static class SpinDexerIOInputs {
    public boolean spinDexerDisconnected = false;
    public double spinDexerMotorVoltage = 0.0;
    public double spinDexerMotorCurrent = 0.0;

    public boolean spinDexerKickerDisconnected = false;
    public double spinDexerKickerMotorVoltage = 0.0;
    public double spinDexerKickerMotorCurrent = 0.0;
  }

  public default void updateInputs(SpinDexerIOInputs inputs) {}

  public default void setSpinDexerSpeed(double speed) {}

  public default void setSpinDexerKickerSpeed(double speed) {}
}
