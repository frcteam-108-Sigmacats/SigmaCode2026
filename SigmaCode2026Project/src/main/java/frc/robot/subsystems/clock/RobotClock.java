package frc.robot.subsystems.clock;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotClock extends SubsystemBase {

  // SmartDashboard
  private static final String DASHBOARD_KEY = "FMS Timer";

  private boolean fmsTimer;

  // Onboard timer — starts when autonomous begins if FMS is unavailable.
  private final Timer robotTimer = new Timer();

  // Tracks whether the robot timer has already been started this enable period.
  private boolean robotTimerStarted = false;

  // The active match/elapsed time in seconds.
  private double currentTime = 0.0;

  public RobotClock() {
    fmsTimer = false;
    SmartDashboard.putBoolean(DASHBOARD_KEY, fmsTimer);
  }

  @Override
  public void periodic() {

    fmsTimer = SmartDashboard.getBoolean(DASHBOARD_KEY, false);

    if (DriverStation.isFMSAttached()) {
      if (fmsTimer) {

        currentTime = DriverStation.getMatchTime();
      } else {

        if (DriverStation.isAutonomousEnabled()) {
          if (!robotTimerStarted) {
            robotTimer.reset();
            robotTimer.start();
            robotTimerStarted = true;
          }
          currentTime = robotTimer.get();
        }
      }
    }

    SmartDashboard.putNumber("Robot Clock/Current Time", currentTime);
    SmartDashboard.putBoolean(
        "Robot Clock/Using FMS Timer", fmsTimer && DriverStation.isFMSAttached());
  }

  public void resetRobotTimer() {
    robotTimer.reset();
    robotTimer.stop();
    robotTimerStarted = false;
  }

  public double getCurrentTime() {
    return currentTime;
  }

  public boolean isUsingFMSTime() {
    return fmsTimer && DriverStation.isFMSAttached();
  }
}
