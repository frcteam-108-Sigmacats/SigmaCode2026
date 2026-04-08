package frc.robot.subsystems.SpinDexer;

public class SpinDexerConstants {
  // IDS for all Components apart of the mechanism
  public static int spinDexerID = 12;
  public static int kicker1ID = 13;
  public static int kicker2ID = 14;
  public static int canRangeID = 19;

  // Current for the SpinDexerMotor
  public static int spinDexerCurrentLimit = 40;

  // Speeds for all components in the mechanism (Units: Percentages)
  public static double SpinDexerClockWise = 1.0;
  public static double SpinDexerCounterClockWise = -0.75;
  public static double KickerForward = 1.0;
  public static double KickerReverse = -0.75;
}
