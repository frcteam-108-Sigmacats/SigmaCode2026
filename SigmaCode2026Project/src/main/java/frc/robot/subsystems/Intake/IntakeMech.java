package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeMech extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Alert rollerDisconnected = new Alert("Roller Motor Is Disconnected", AlertType.kError);

  public IntakeMech(IntakeIO io) {
    this.io = io;
  }

  public void setIntakeRoller() {
    io.setIntakeRollerSpeed(IntakeConstants.rollerSpeedPercentage);
  }

  public void setOuttakeRoller() {
    io.setIntakeRollerSpeed(IntakeConstants.outtakeSpeedPercentage);
  }

  public void stopRollerMotor() {
    io.setRollerVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeMech/", inputs);
    rollerDisconnected.set(inputs.intakeRollerDisconnected);
  }
}
