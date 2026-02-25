package frc.robot.subsystems.Intake;

public class IntakeConstants {
  // CAN IDs for the Motors
  public static int actuaterID = 10;
  public static int rollerID = 11;
  // Current Limits for the Motors
  public static int actuaterCurrentLimit = 40;
  public static int rollerCurrentLimit = 40;
  // PID Values for the Actuation Controller
  public static double actuaterP = 0.0;
  public static double actuaterI = 0.0;
  public static double actuaterD = 0.0;
  // Encoder Configuration for Actuator Controller
  public static double actuaterPositionConversion = 16;
  public static boolean actucaterEncoderInverted = false;
  // Intake State Values
  public static double intakeOutPosition = 70;
  public static double intakeInPosition = 0;
  public static double rollerSpeedPercentage = 0;
}
