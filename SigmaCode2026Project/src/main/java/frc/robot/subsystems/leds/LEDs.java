package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.LEDSetting;
import java.util.Optional;

public class LEDs extends SubsystemBase {

  // ── Singleton ──────────────────────────────────────────────────────────────

  private static LEDs m_instance;

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  // ── Hardware ───────────────────────────────────────────────────────────────
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  // ── Patterns ───────────────────────────────────────────────────────────────

  private final LEDPattern blue = LEDPattern.solid(Color.kBlue);

  private final LEDPattern red = LEDPattern.solid(Color.kRed);
  // Shift warning → solid yellow  (transition coming in ≤5 seconds)
  private final LEDPattern yellow = LEDPattern.solid(Color.kYellow);

  // ── State ──────────────────────────────────────────────────────────────────
  private boolean change = false;
  private LEDSetting ledMode;

  private LEDSetting lastHubSetting = null;

  private static final double[] SHIFT_BOUNDARIES = {130.0, 105.0, 80.0, 55.0, 30.0};

  private static final double WARNING_WINDOW = 5.0;

  // ── Constructor ────────────────────────────────────────────────────────────

  public LEDs() {
    m_led = new AddressableLED(9);
    m_buffer = new AddressableLEDBuffer(LEDConstants.k_stripLength);
    m_led.setLength(LEDConstants.k_stripLength);
    m_led.start();
  }

  // ── Periodic ───────────────────────────────────────────────────────────────

  @Override
  public void periodic() {

    updateHubLEDFromGameData();

    if (change) {
      switch (ledMode) {
        case BLUE:
          blue.applyTo(m_buffer);
          break;
        case HUB_ACTIVE:
          blue.applyTo(m_buffer);
          break;
        case HUB_WARNING:
          yellow.applyTo(m_buffer);
          break;
        case HUB_INACTIVE:
          red.applyTo(m_buffer);
          break;
      }
      m_led.setData(m_buffer);
      change = false;
    }
  }

  private void updateHubLEDFromGameData() {
    LEDSetting computed = computeHubLEDSetting();
    if (computed != null && computed != lastHubSetting) {
      lastHubSetting = computed;
      ledMode = computed;
      change = true;
    }
  }

  private LEDSetting computeHubLEDSetting() {

    if (DriverStation.isAutonomousEnabled()) {
      return LEDSetting.HUB_ACTIVE;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return null;
    }

    double matchTime = DriverStation.getMatchTime();

    if (matchTime < 0) {
      return LEDSetting.HUB_ACTIVE;
    }

    if (matchTime > SHIFT_BOUNDARIES[0] || matchTime <= SHIFT_BOUNDARIES[4]) {

      if (isWithinWarningWindow(matchTime)) {
        return LEDSetting.HUB_WARNING;
      }
      return LEDSetting.HUB_ACTIVE;
    }

    if (isWithinWarningWindow(matchTime)) {
      return LEDSetting.HUB_WARNING;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {

      return LEDSetting.HUB_ACTIVE;
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return LEDSetting.HUB_ACTIVE;
    }

    boolean redInactiveFirst = gameData.charAt(0) == 'R';

    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    boolean hubActive = isHubActiveInCurrentShift(matchTime, shift1Active);
    return hubActive ? LEDSetting.HUB_ACTIVE : LEDSetting.HUB_INACTIVE;
  }

  private boolean isWithinWarningWindow(double matchTime) {
    for (double boundary : SHIFT_BOUNDARIES) {
      if (matchTime > boundary && matchTime <= boundary + WARNING_WINDOW) {
        return true;
      }
    }
    return false;
  }

  private boolean isHubActiveInCurrentShift(double matchTime, boolean shift1Active) {
    if (matchTime > SHIFT_BOUNDARIES[1]) return shift1Active; // Shift 1: > 105
    if (matchTime > SHIFT_BOUNDARIES[2]) return !shift1Active; // Shift 2: > 80
    if (matchTime > SHIFT_BOUNDARIES[3]) return shift1Active; // Shift 3: > 55
    if (matchTime > SHIFT_BOUNDARIES[4]) return !shift1Active; // Shift 4: > 30
    return true; // End game: ≤ 30
  }

  public void changeLEDMode(LEDSetting mode) {
    ledMode = mode;
    change = true;
  }

  // Returns the currently active LED setting.
  public LEDSetting grabLEDMode() {
    return ledMode;
  }
}
