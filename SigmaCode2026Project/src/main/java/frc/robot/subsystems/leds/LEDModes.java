package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Static LED pattern helpers. Each method writes directly into the provided buffer segment. */
public final class LEDModes {

  private LEDModes() {}

  // ── Solid color ────────────────────────────────────────────────────────────

  /** Fills [start, start+length) with a single solid color. */
  public static void solid(AddressableLEDBuffer buffer, int start, int length, Color color) {
    for (int i = start; i < start + length; i++) {
      buffer.setLED(i, color);
    }
  }

  // ── Red / Blue hard switch ─────────────────────────────────────────────────

  private static final double RED_BLUE_INTERVAL_S = 0.4; // seconds per color

  /**
   * Switches the entire strip between solid red and solid blue on a fixed timer. This replicates
   * the "switching between red and blue" behavior shown in the robot video.
   */
  public static void redBlueSwitch(AddressableLEDBuffer buffer, int start, int length) {
    double t = System.currentTimeMillis() / 1000.0;
    boolean showRed = ((int) (t / RED_BLUE_INTERVAL_S) % 2) == 0;
    solid(buffer, start, length, showRed ? Color.kRed : Color.kBlue);
  }

  // ── Red / Blue split (sides swap on same timer) ────────────────────────────

  /**
   * Divides the strip in half: one side red, one side blue. The two halves swap colors every {@code
   * RED_BLUE_INTERVAL_S} seconds.
   */
  public static void redBlueSplit(AddressableLEDBuffer buffer, int start, int length) {
    double t = System.currentTimeMillis() / 1000.0;
    boolean swapped = ((int) (t / RED_BLUE_INTERVAL_S) % 2) == 0;
    int half = length / 2;
    solid(buffer, start, half, swapped ? Color.kRed : Color.kBlue);
    solid(buffer, start + half, length - half, swapped ? Color.kBlue : Color.kRed);
  }

  // ── Breathe ────────────────────────────────────────────────────────────────

  private static final double BREATHE_SPEED = 1.5; // cycles per second
  private static final int BREATHE_MIN = 20; // minimum brightness (0-255)

  /** Smoothly pulses the strip brightness in the given color. */
  public static void breathe(AddressableLEDBuffer buffer, int start, int length, Color color) {
    double phase = System.currentTimeMillis() / 1000.0 * BREATHE_SPEED * Math.PI;
    int brightness = (int) (((Math.sin(phase) + 1.0) / 2.0) * (255 - BREATHE_MIN) + BREATHE_MIN);

    int r = (int) (color.red * brightness);
    int g = (int) (color.green * brightness);
    int b = (int) (color.blue * brightness);

    for (int i = start; i < start + length; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

  // ── Rainbow ────────────────────────────────────────────────────────────────

  private static final double RAINBOW_SPEED = 80.0; // hue units per second

  /** Scrolling rainbow across the strip. */
  public static void rainbow(AddressableLEDBuffer buffer, int start, int length) {
    int firstHue = (int) ((System.currentTimeMillis() / 1000.0 * RAINBOW_SPEED) % 180);
    for (int i = start; i < start + length; i++) {
      int hue = (firstHue + (i * 180 / length)) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
  }

  // ── Off ────────────────────────────────────────────────────────────────────

  /** Turns all LEDs off. */
  public static void off(AddressableLEDBuffer buffer, int start, int length) {
    solid(buffer, start, length, Color.kBlack);
  }
}
