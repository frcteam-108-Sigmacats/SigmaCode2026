package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeMech extends SubsystemBase {
  // Constructs the general interface that will be used for the mechanism
  private IntakeIO io;
  // Constructs the inputs that will be logged for this mechanism
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  // Creates an Alert to check if the speed controller gets disconnected
  private Alert rollerDisconnected = new Alert("Roller Motor Is Disconnected", AlertType.kError);

  /**
   * Constructs the intake mechanism
   *
   * @param io Interface can be SIM or REAL
   */
  public IntakeMech(IntakeIO io) {
    this.io = io;
  }
  /** Sets the roller to intake the fuel */
  public void setIntakeRoller() {
    io.setIntakeRollerSpeed(IntakeConstants.rollerSpeedPercentage);
  }
  /** Sets the roller to outtake the fuel */
  public void setOuttakeRoller() {
    io.setIntakeRollerSpeed(IntakeConstants.outtakeSpeedPercentage);
  }
  /** Stops the roller using 0 voltage */
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
