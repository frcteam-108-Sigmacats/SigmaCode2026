package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.LEDSetting;

public class LEDs extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private final LEDPattern blue = LEDPattern.solid(Color.kBlue);

  private boolean change = false;
  private LEDSetting ledMode;

  public LEDs() {
    m_led = new AddressableLED(9);
    m_buffer = new AddressableLEDBuffer(LEDConstants.k_stripLength);
    m_led.setLength(LEDConstants.k_stripLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    if (change) {
      switch (ledMode) {
        case BLUE:
          blue.applyTo(m_buffer);
          break;
      }
      m_led.setData(m_buffer);
      change = false;
    }
  }

  public void changeLEDMode(LEDSetting mode) {
    ledMode = mode;
    change = true;
  }

  public LEDSetting grabLEDMode() {
    return ledMode;
  }
}
