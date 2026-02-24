package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SpinDexer.SpinDexerIO;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class SpindexerMech extends SubsystemBase {
    private SpinDexerIO io;
    private Alert kicker1Disconnected = new Alert("Kicker1 Motor is Disconnected", AlertType.kError);
    private Alert kicker2Disconnected = new Alert("Kicker2 Motor is Disconnected", AlertType.kError);
    public SpindexerMech(SpinDexerIO io) {
      this.io = io;
    }

    public void setSpindexerLeft(){
      io.setSpinDexerKickerSpeed(0);
    }
    public void setSpindexerRight(){
      io.setSpinDexerKickerSpeed(0);
    }
    public void setKickerForward(){
      io.setKickerForward(0);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

