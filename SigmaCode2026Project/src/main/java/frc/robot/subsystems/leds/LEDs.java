package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  private final int m_length = Constants.LEDsConstants.k_totalLength;

  // ── Mode ───────────────────────────────────────────────────────────────────

  /** All available LED modes. */
  public enum Mode {
    RED_BLUE_SWITCH, // full-strip alternates red ↔ blue  (matches the robot video)
    RED_BLUE_SPLIT, // half red / half blue, sides swap
    BREATHE_RED,
    BREATHE_BLUE,
    SOLID_RED,
    SOLID_BLUE,
    RAINBOW,
    OFF
  }

  private Mode m_mode = Mode.OFF;

  // ── Constructor ────────────────────────────────────────────────────────────

  private LEDs() {
    m_led = new AddressableLED(Constants.LEDsConstants.k_PWMId);
    m_buffer = new AddressableLEDBuffer(m_length);
    m_led.setLength(m_length);
    m_led.start();
  }

  // ── Periodic ───────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    switch (m_mode) {
      case RED_BLUE_SWITCH -> LEDModes.redBlueSwitch(m_buffer, 0, m_length);
      case RED_BLUE_SPLIT -> LEDModes.redBlueSplit(m_buffer, 0, m_length);
      case BREATHE_RED -> LEDModes.breathe(m_buffer, 0, m_length, Color.kRed);
      case BREATHE_BLUE -> LEDModes.breathe(m_buffer, 0, m_length, Color.kBlue);
      case SOLID_RED -> LEDModes.solid(m_buffer, 0, m_length, Color.kRed);
      case SOLID_BLUE -> LEDModes.solid(m_buffer, 0, m_length, Color.kBlue);
      case RAINBOW -> LEDModes.rainbow(m_buffer, 0, m_length);
      case OFF -> LEDModes.off(m_buffer, 0, m_length);
    }
    m_led.setData(m_buffer);
  }

  // ── Mode setters ───────────────────────────────────────────────────────────

  /**
   * Full strip switches between red and blue every ~0.4 s (matches the robot video)[i dont know how
   * to access the fms to do this].
   */
  public void setRedBlueSwitch() {
    m_mode = Mode.RED_BLUE_SWITCH;
  }

  /** Strip is split half red / half blue; the two sides swap on the same timer. */
  public void setRedBlueSplit() {
    m_mode = Mode.RED_BLUE_SPLIT;
  }

  public void setBreatheRed() {
    m_mode = Mode.BREATHE_RED;
  }

  public void setBreatheBlue() {
    m_mode = Mode.BREATHE_BLUE;
  }

  public void setSolidRed() {
    m_mode = Mode.SOLID_RED;
  }

  public void setSolidBlue() {
    m_mode = Mode.SOLID_BLUE;
  }

  public void setRainbow() {
    m_mode = Mode.RAINBOW;
  }

  public void setOff() {
    m_mode = Mode.OFF;
  }

  public void setMode(Mode mode) {
    m_mode = mode;
  }

  public Mode getMode() {
    return m_mode;
  }
}
