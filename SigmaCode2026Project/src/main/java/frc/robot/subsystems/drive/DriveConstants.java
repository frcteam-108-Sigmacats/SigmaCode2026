// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TrajectoryMap;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.6; // Max 5.07 m/s
  public static final double slowSpeedMetersPerSec = 1.0;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(21.5);
  public static final double wheelBase = Units.inchesToMeters(25.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

  // Device CAN IDs
  public static final int pigeonCanId = 1;

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 5;
  public static final int frontRightDriveCanId = 3;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 6;
  public static final int frontRightTurnCanId = 4;
  public static final int backRightTurnCanId = 8;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double driveMotorReduction =
      (4.71); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.05;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.1;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 50;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 66.86; // 110lbs
  public static final double robotMOI = 5.318;
  public static final double wheelCOF = 1.6;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              0.038,
              5.07,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));

  // Limelights
  public static final String kLimelightBackLeftName = "limelight-bl";
  public static final String kLimelightBackRightName = "limelight-br";
  // public static final String kLimelightFrontName = "limelight-tf"; // tf = front limelight

  public static double xyStdDevCoefficient = 0.005;
  public static double thetaStdDevCoefficient = 0.01;
  public static double stdDevFactor = 0.5;
  public static boolean useVisionRotation = true;
  public static double autoStdDevScale = 0;

  // Constants for AUTO
  public static PIDController xyLinearPIDController = new PIDController(0, 0, 0);
  public static ProfiledPIDController thetaPIDController =
      new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  //Auto Paths
  //Depot Paths
  public static TrajectoryMap depotPath1Pose1 = new TrajectoryMap(new Pose2d(3.5, 5.6, Rotation2d.fromDegrees(-45)), Rotation2d.fromDegrees(-45), 1.5);
  public static TrajectoryMap depotPath1Pose2 = new TrajectoryMap(new Pose2d(6.3, 5.6, Rotation2d.fromDegrees(-45)), Rotation2d.fromDegrees(-45), 1.5);
  public static TrajectoryMap depotPath2Pose1 = new TrajectoryMap(new Pose2d(7.495, 6.684, Rotation2d.fromDegrees(-45)), Rotation2d.fromDegrees(-45), 3);
  public static TrajectoryMap depotPath2Pose2 = new TrajectoryMap(new Pose2d(7.8, 4.5, Rotation2d.fromDegrees(-90)), Rotation2d.fromDegrees(-90), 0.75);
  public static TrajectoryMap depotPath3Pose1 = new TrajectoryMap(new Pose2d(6.0, 5.5, Rotation2d.fromDegrees(-135)), Rotation2d.fromDegrees(-135), 4.8);
  public static TrajectoryMap depotPath3Pose2 = new TrajectoryMap(new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(-135)), Rotation2d.fromDegrees(-135), 1.5);
  //Station Paths
  public static TrajectoryMap stationPath1Pose1 = new TrajectoryMap(new Pose2d(3.418, 2.407, Rotation2d.fromDegrees(45)), Rotation2d.fromDegrees(45), 3.0);
  public static TrajectoryMap stationPath1Pose2 = new TrajectoryMap(new Pose2d(6.048, 2.407, Rotation2d.fromDegrees(45)), Rotation2d.fromDegrees(45), 3.0);
  public static TrajectoryMap stationPath2Pose1 = new TrajectoryMap(new Pose2d(7.605, 0.944, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(45), 3.0);
  public static TrajectoryMap stationPath2Pose2 = new TrajectoryMap(new Pose2d(7.848, 3.394, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(45), 0.75);
  public static TrajectoryMap stationPath3Pose1 = new TrajectoryMap(new Pose2d(6.177, 2.549, Rotation2d.fromDegrees(-135)), Rotation2d.fromDegrees(-135), 4.8);
  public static TrajectoryMap stationPath3Pose2 = new TrajectoryMap(new Pose2d(3.5, 2.549, Rotation2d.fromDegrees(-135)), Rotation2d.fromDegrees(-135), 2);
  public static TrajectoryMap stationPath4Pose1 = new TrajectoryMap(new Pose2d(0.806, 0.652, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180), 2.0);

  //Path Groups
  public static List<TrajectoryMap> depotPath1 = List.of(depotPath1Pose1, depotPath1Pose2);
  public static List<TrajectoryMap> depotPath2 = List.of(depotPath2Pose1, depotPath2Pose2);
  public static List<TrajectoryMap> depotPath3 = List.of(depotPath3Pose1, depotPath3Pose2);

  public static List<TrajectoryMap> stationPath1 = List.of(stationPath1Pose1, stationPath1Pose2);
  public static List<TrajectoryMap> stationPath2 = List.of(stationPath2Pose1, stationPath2Pose2);
  public static List<TrajectoryMap> stationPath3 = List.of(stationPath3Pose1, stationPath3Pose2);
  public static List<TrajectoryMap> stationPath4 = List.of(stationPath4Pose1);

  //Paths to test and modify PID Controllers for Holonomic Drive
  public static TrajectoryMap testPath1Pose1 = new TrajectoryMap(new Pose2d(0,0, Rotation2d.kZero), Rotation2d.kZero, 2.0);
  public static TrajectoryMap testPath1Pose2 = new TrajectoryMap(new Pose2d(1, 0, Rotation2d.kZero), Rotation2d.kZero, 2.0);
  public static TrajectoryMap testPath1Pose3 = new TrajectoryMap(new Pose2d(1, 1, Rotation2d.kZero), Rotation2d.kZero, 2.0);
  public static TrajectoryMap testPath1Pose4 = new TrajectoryMap(new Pose2d(0, 1, Rotation2d.kZero), Rotation2d.kZero, 2.0);
  public static TrajectoryMap testPath1Pose5 = new TrajectoryMap(new Pose2d(0, 0, Rotation2d.kZero), Rotation2d.kZero, 2.0);

  public static List<TrajectoryMap> testPath = List.of(testPath1Pose1, testPath1Pose2, testPath1Pose3, testPath1Pose4, testPath1Pose5);
}
