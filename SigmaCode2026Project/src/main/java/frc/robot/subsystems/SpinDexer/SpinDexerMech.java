package frc.robot.subsystems.SpinDexer;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterConstants.*;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SpinDexerMech extends SubsystemBase {
  // Creating the general interface that will run either the hardware or simulated hardware code
  private SpinDexerIO io;
  // Creating the inputs that will be logged and grabbed from this mechanism
  private final SpinDexerIOInputsAutoLogged inputs = new SpinDexerIOInputsAutoLogged();

  // Created 2 counter variables to keep track of the balls we shoot and pass
  private LoggedNetworkNumber shootCount = new LoggedNetworkNumber("/Counter/ShotCount", 0);
  private LoggedNetworkNumber passCount = new LoggedNetworkNumber("/Counter/PassingCount", 0);
  // Keeps track of the state that the robot is in for counting purposes
  public ShooterStatus shooterState = ShooterStatus.SHOOT;
  // Checks if the initial ball detection has happened for accurate counting
  private boolean detectedBall = false;
  // Checks if the robot has ever been enabled, purpose is for logging ball counting data into a
  // JSON File
  private boolean hasRobotBeenEnabled = false;
  // Alerts that will keep track if the speed controllers ever get disconnected
  private Alert spinDexerDisconnected = new Alert("SpinDexer Motor Disconnected", AlertType.kError);
  private Alert kicker1Disconnected = new Alert("Kicker1 Motor is Disconnected", AlertType.kError);
  private Alert kicker2Disconnected = new Alert("Kicker2 Motor is Disconnected", AlertType.kError);

  // GSON used to transfer data from a HashMap to a JSON format
  private Gson gson = new Gson();
  // A HashMap to keep track of the data per match
  private Map<String, Object> infoMap = new HashMap<>();

  /**
   * Constructs the general inferface with the specific given interface
   *
   * @param io Interface can either be SIM or REAL
   */
  public SpinDexerMech(SpinDexerIO io) {
    this.io = io;
  }

  /**
   * Sets the SpinDexer Motor to rotate Counter Clockwise when looking at it from the top
   * perspective
   */
  public void setSpinDexerClockwise() {
    io.setSpinDexerSpeed(SpinDexerConstants.SpinDexerClockWise);
  }
  /**
   * Sets the Kickers to CounterClockwise where it is removing fuel from the shooter and into the
   * hopper
   */
  public void setKickerReverse() {
    io.setKickerSpeed(SpinDexerConstants.KickerReverse);
  }
  /** Sets the Spin Dexer to rotate ClockWise when looking at it from the top perspective */
  public void setSpinDexerCounterClockwise() {
    io.setSpinDexerSpeed(SpinDexerConstants.SpinDexerCounterClockWise);
  }
  /** Sets the Kickers to rotate Clockwise to move fuel from the hopper to the shooter */
  public void setKickerForward() {
    io.setKickerSpeed(SpinDexerConstants.KickerForward);
  }
  /** Stops the SpinDexer by setting SpinDexer Voltage to 0 */
  public void stopSpinDexer() {
    io.setSpinDexerVoltage(0);
  }
  /** Stops the Kicker Motors by setting the motors to 0 Voltage */
  public void stopKickers() {
    io.setKickerVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updating all inputs to whatever the Interface data is given us
    io.updateInputs(inputs);
    Logger.processInputs("SpinDexerMech/", inputs);
    // Checks if we are detecting a ball
    if (inputs.detectBall) {
      detectedBall = true;
    } else {
      // If ball is not detected but was detected, add one to shoot or pass ball count
      if (detectedBall) {
        if (shooterState == ShooterStatus.SHOOT) {
          shootCount.set(shootCount.get() + 1);
        } else if (shooterState == ShooterStatus.PASSING) {
          passCount.set(passCount.get() + 1);
        }
        // Resets for the next detection of a ball
        detectedBall = false;
      }
    }
    // Checks to see if we are in a Match (connected to FMS) to be able to log data at the end of
    // the match
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.isEnabled()) {
        hasRobotBeenEnabled = true;
      } else {
        if (hasRobotBeenEnabled) {
          // Puts all needed information into the Hash Map
          infoMap.put("Match Type", DriverStation.getMatchType().toString());
          infoMap.put("Match Number", DriverStation.getMatchNumber());
          infoMap.put("Shoot Count", shootCount.get());
          infoMap.put("Passing Count", passCount.get());
          // Packages the info map into a JSON Format
          String jsonPackage = gson.toJson(infoMap);
          // The directory of where the JSON file resides in the USB Drive in the Roborio
          String filename = "/u/soflocomp.json";
          // Try Catch used to make sure we are able to try to write the JSON data into the file
          // without causing an error to pop up on our robot
          try (FileWriter fileWriter = new FileWriter(filename, true)) {
            fileWriter.write(jsonPackage);
            System.out.println("Match Data saved to " + filename);
          } catch (Exception e) {
            System.err.println("Failed to write into JSON File: " + e.getMessage());
          }
        }
      }
    }
  }
  /**
   * Sets the shooter status of our robot
   *
   * @param status SHOOT or PASSING or INTAKE or DRIVE
   */
  public void setShooterStatus(ShooterStatus status) {
    shooterState = status;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
