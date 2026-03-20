package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Seconds;

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

  // ── Hardware ───────────────────────────────────────────────────────────────
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  // ── Base solid patterns ────────────────────────────────────────────────────
  private final LEDPattern blue = LEDPattern.solid(Color.kBlue);
  private final LEDPattern red = LEDPattern.solid(Color.kRed);
  private final LEDPattern yellow = LEDPattern.solid(Color.kYellow);

  // ── Animated patterns ──────────────────────────────────────────────────────

  private final LEDPattern bluePulse = blue.breathe(Seconds.of(1.0));

  private final LEDPattern redPulse = red.breathe(Seconds.of(1.0));

  private final LEDPattern yellowBlink = yellow.blink(Seconds.of(0.2));

  // ── State ──────────────────────────────────────────────────────────────────
  private boolean change = false;
  private LEDSetting ledMode = LEDSetting.BLUE;

  private LEDPattern currentPattern;

  private LEDSetting lastHubSetting = null;

  // ── Shift timing (match time counts DOWN from 135 s) ──────────────────────

  private static final double[] SHIFT_BOUNDARIES = {130.0, 105.0, 80.0, 55.0, 30.0};

  private static final double ACTIVE_WARNING_WINDOW = 2.5;

  // ── Constructor ────────────────────────────────────────────────────────────

  public LEDs() {
    m_led = new AddressableLED(9);
    m_buffer = new AddressableLEDBuffer(LEDConstants.k_stripLength);
    m_led.setLength(LEDConstants.k_stripLength);
    m_led.start();
    currentPattern = blue;
  }

  // ── Periodic ───────────────────────────────────────────────────────────────

  @Override
  public void periodic() {

    updateHubLEDFromGameData();

    currentPattern.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  // ── Hub state detection ────────────────────────────────────────────────────

  private void updateHubLEDFromGameData() {
    LEDSetting computed = computeHubLEDSetting();
    if (computed != null && computed != lastHubSetting) {
      lastHubSetting = computed;
      ledMode = computed;
      currentPattern = patternFor(computed);
      change = true;
    }
  }

  private LEDPattern patternFor(LEDSetting setting) {
    switch (setting) {
      case HUB_ACTIVE:
        return bluePulse;
      case HUB_WARNING:
        return yellowBlink;
      case HUB_INACTIVE:
        return redPulse;
      case BLUE:
      default:
        return blue;
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

    if (matchTime > SHIFT_BOUNDARIES[0] || matchTime <= SHIFT_BOUNDARIES[4]) {
      return LEDSetting.HUB_ACTIVE;
    }

    boolean currentlyActive = isHubActiveInCurrentShift(matchTime, shift1Active);

    if (!currentlyActive && isWithinActiveWarningWindow(matchTime, shift1Active)) {
      return LEDSetting.HUB_WARNING;
    }

    return currentlyActive ? LEDSetting.HUB_ACTIVE : LEDSetting.HUB_INACTIVE;
  }

  private boolean isWithinActiveWarningWindow(double matchTime, boolean shift1Active) {
    for (double boundary : SHIFT_BOUNDARIES) {
      boolean nextShiftActive = isHubActiveInCurrentShift(boundary - 0.01, shift1Active);
      if (nextShiftActive) {
        if (matchTime > boundary && matchTime <= boundary + ACTIVE_WARNING_WINDOW) {
          return true;
        }
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
    currentPattern = patternFor(mode);
    change = true;
  }

  public LEDSetting grabLEDMode() {
    return ledMode;
  }
}
