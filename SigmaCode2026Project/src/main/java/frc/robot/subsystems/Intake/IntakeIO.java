package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeRollerDisconnected = false;
    public double intakeRollerCurrent = 0.0;
    public double intakeRollerVoltage = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeRollerSpeed(double speed) {}

  public default void setRollerVoltage(double voltage) {}
}
