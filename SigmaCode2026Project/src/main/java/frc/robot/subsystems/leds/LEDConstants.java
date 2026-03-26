package frc.robot.subsystems.leds;

public class LEDConstants {

  public static final int k_stripLength = 37;

  /**
   * All LED display settings.
   *
   * <p>Hub states are driven automatically in LEDs.periodic() from FMS game data:
   *
   * <ul>
   *   <li>{@link #BLUE} — manual solid blue (pre-match / default)
   *   <li>{@link #HUB_ACTIVE} — hub is open: pulsing blue (breathe)
   *   <li>{@link #HUB_WARNING} — 2.5 s before hub goes active: blinking yellow. Start shooting now
   *       — the ball will score when the hub opens.
   *   <li>{@link #HUB_INACTIVE} — hub is closed this shift: pulsing red (breathe)
   * </ul>
   */
  public enum LEDSetting {
    BLUE,
    HUB_ACTIVE,
    HUB_WARNING,
    HUB_INACTIVE
  }
}
