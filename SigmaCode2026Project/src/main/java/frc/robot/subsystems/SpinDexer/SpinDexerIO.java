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

    public boolean detectBall = false;
  }

  public default void updateInputs(SpinDexerIOInputs inputs) {}
  /**
   * Setting the speed percentage of the SpinDexer
   *
   * @param speed The speed percentage (-1.0 to 1.0)
   */
  public default void setSpinDexerSpeed(double speed) {}
  /**
   * Setting the speed percentage of the 2 Kicker Motors
   *
   * @param speed The speed percentage (-1.0 to 1.0)
   */
  public default void setKickerSpeed(double speed) {}
  /**
   * Setting the voltage of the SpinDexer Motor
   *
   * @param volt Voltage between -12V to 12V
   */
  public default void setSpinDexerVoltage(double volt) {}
  /**
   * Setting the voltage of the 2 Kicker Motors
   *
   * @param volt Voltage between -12V to 12V
   */
  public default void setKickerVoltage(double volt) {}
}
