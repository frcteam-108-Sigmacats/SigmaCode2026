package frc.robot;

public class Helpers {

  public static int clamp(int val, int min, int max) {
    return Math.max(min, Math.min(max, val));
  }
}
