package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeActuaterDisconnected = false;
    public double intakeActuaterCurrent = 0.0;
    public double intakeActuaterVoltage = 0.0;
    public Rotation2d intakeActuaterAngle = new Rotation2d();

    public boolean intakeRollerDisconnected = false;
    public double intakeRollerCurrent = 0.0;
    public double intakeRollerVoltage = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeActuaterSpeed(double speed) {}

  public default void setIntakeActuaterPosition(double position) {}

  public default void setIntakeRollerSpeed(double speed) {}

  public default void setActuaterVoltage(double voltage) {}

  public default void setRollerVoltage(double voltage) {}
}
