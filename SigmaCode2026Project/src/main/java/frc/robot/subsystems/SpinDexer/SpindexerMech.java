package frc.robot.subsystems.SpinDexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterConstants.*;

import java.io.FileWriter;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.google.gson.Gson;


public class SpinDexerMech extends SubsystemBase {
  private SpinDexerIO io;
  private final SpinDexerIOInputsAutoLogged inputs = new SpinDexerIOInputsAutoLogged();

  private LoggedNetworkNumber shootCount = new LoggedNetworkNumber("/Counter", 0);
  private LoggedNetworkNumber passCount = new LoggedNetworkNumber("/Counter", 0);
  public ShooterStatus shooterState = ShooterStatus.SHOOT;

  private boolean detectedBall = false;

  private boolean hasRobotBeenEnabled = false;
  private Alert spinDexerDisconnected = new Alert("SpinDexer Motor Disconnected", AlertType.kError);
  private Alert kicker1Disconnected = new Alert("Kicker1 Motor is Disconnected", AlertType.kError);
  private Alert kicker2Disconnected = new Alert("Kicker2 Motor is Disconnected", AlertType.kError);

  private Gson gson = new Gson();

  private Map<String, Object> infoMap = new HashMap<>();

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
    if(inputs.detectBall){
      detectedBall = true;
    }
    else{
      if(detectedBall){
        if(shooterState == ShooterStatus.SHOOT){
        shootCount.set(shootCount.get() + 1);
        }
        else if(shooterState == ShooterStatus.PASSING){
          passCount.set(passCount.get() + 1);
        }
        detectedBall = false;
      }
    }
    if(DriverStation.isFMSAttached()){
      if(DriverStation.isEnabled()){
        hasRobotBeenEnabled = true;
      }
      else{
        if(hasRobotBeenEnabled){
          infoMap.put("Match Type", DriverStation.getMatchType().toString());
          infoMap.put("Match Number", DriverStation.getMatchNumber());
          infoMap.put("Shoot Count", shootCount.get());
          infoMap.put("Passing Count", passCount.get());
          String jsonPackage = gson.toJson(infoMap);
          String filename = "/u/soflocomp.json";
          try(FileWriter fileWriter = new FileWriter(filename, true)){
            fileWriter.write(jsonPackage);
            System.out.println("Match Data saved to " + filename);
          }catch (Exception e) {
            System.err.println("Failed to write into JSON File: " + e.getMessage());
          }
        }
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
