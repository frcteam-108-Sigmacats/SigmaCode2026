package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeRollerDisconnected = false;
    public double intakeRollerCurrent = 0.0;
    public double intakeRollerVoltage = 0.0;

    public boolean intakeRoller2Disconnected = false;
    public double intakeRoller2Current = 0.0;
    public double intakeRoller2Voltage = 0.0;
  }
  /**
   * Updates all the loggable inputs with the given values
   *
   * @param inputs the loggable inputs from {@link IntakeIOInputs}
   */
  public default void updateInputs(IntakeIOInputs inputs) {}
  /**
   * Sets the intake roller with speed
   *
   * @param speed -1.0 to +1.0
   */
  public default void setIntakeRollerSpeed(double speed) {}
  /**
   * Sets the Intake Roller to a voltage
   *
   * @param voltage -12V to +12V
   */
  public default void setRollerVoltage(double voltage) {}
}
