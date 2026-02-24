package frc.robot.subsystems.SpinDexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SpinDexerMech extends SubsystemBase {
  private SpinDexerIO io;
  private final SpinDexerIOInputsAutoLogged inputs = new SpinDexerIOInputsAutoLogged();
  private Alert spinDexerDisconnected = new Alert("SpinDexer Motor Disconnected", AlertType.kError);
  private Alert kicker1Disconnected = new Alert("Kicker1 Motor is Disconnected", AlertType.kError);
  private Alert kicker2Disconnected = new Alert("Kicker2 Motor is Disconnected", AlertType.kError);

  public SpinDexerMech(SpinDexerIO io) {
    this.io = io;
  }

  public void setSpinDexerClockwise() {
    io.setSpinDexerSpeed(SpinDexerConstants.SpinDexerClockWise);
  }

  public void setSpinDexerCounterClockwise() {
    io.setSpinDexerSpeed(SpinDexerConstants.SpinDexerCounterClockWise);
  }

  public void setKickerForward() {
    io.setKickerSpeed(SpinDexerConstants.KickerForward);
  }

  public void stopSpinDexer() {
    io.setSpinDexerVoltage(0);
  }

  public void stopKickers() {
    io.setKickerVoltage(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("SpinDexerMech/", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
