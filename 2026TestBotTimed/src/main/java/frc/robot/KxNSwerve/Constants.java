// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.KxNSwerve;

import static edu.wpi.first.units.Units.Rotation;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveDriveConstants{
    public static final int fLDriveMotorID = 1;
    public static final int fLTurnMotorID = 2;
    public static final double fLAbsEncoderOffset = -Math.PI / 2;

    public static final int fRDriveMotorID = 3;
    public static final int fRTurnMotorID = 4;
    public static final double fRAbsEncoderOffset = 0;

    public static final int bLDriveMotorID = 5;
    public static final int bLTurnMotorID = 6;
    public static final double bLAbsEncoderOffset = Math.PI;

    public static final int bRDriveMotorID = 7;
    public static final int bRTurnMotorID = 8;
    public static final double bRAbsEncoderOffset = Math.PI / 2;

    public static final int driveMotorCurrentLimit = 50; /*in amps */
    public static final int turnmotorCurrentLimit = 20; /*in amps */

    public static final double krakenRPM = 6000.0;

    public static final int DriveMotorPinionTeeth = 14;

    public static final double wheelDiameter = Units.inchesToMeters(3);

    public static final double kDrivingMotorReduction = (45.0 * 21) / (DriveMotorPinionTeeth * 15);
// p0.9 i0.0 d0.0//
    public static final double drivemotorP = 0.8;
    public static final double drivemotorI = 0.0;
    public static final double drivemotorD = 0.0;
// p1 i0 d0//
    public static final double turnmotorP = 1.0;
    public static final double turnmotorI = 0.0;
    public static final double turnmotorD = 0.0;

    public static final double turningFactor = 2 * Math.PI;

    public static double trackWidth = Units.inchesToMeters(22.4375);
    public static double wheelbase = Units.inchesToMeters(22.4375);
    public static double kMaxSpeedMPS = 21;
    public static double kSlowSpeedMPS = 2;
    public final static double maxAngularspeed = 4 * Math.PI;
    public static boolean gyroReversed = false;
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelbase / 2.0, trackWidth/ 2.0),
      new Translation2d(wheelbase / 2.0, -trackWidth/ 2.0),
      new Translation2d(-wheelbase / 2.0, trackWidth/ 2.0),
      new Translation2d(-wheelbase / 2.0, -trackWidth/ 2.0));
    
      public static final double deadband = 0.2;
  }
  

}

