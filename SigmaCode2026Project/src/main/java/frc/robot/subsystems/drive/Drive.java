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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Shooter.ShooterConstants.ShooterStatus;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final double ODOMETRY_FREQUENCY = new CANBus("PhoenixBus").isNetworkFD() ? 250.0 : 100.0;
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private boolean slowSpeedEnable = false;

  private boolean hasStoppedModules = false;

  // Limelight camera streams
  private HttpCamera limelightBackLeft;
  private HttpCamera limelightBackRight;
  private HttpCamera limelightFront;

  private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();

  private ShooterStatus driveMode = ShooterStatus.DRIVE;

  private static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private static Rotation2d rawGyroRotation = new Rotation2d();

  private Field2d field = new Field2d();

  private boolean isFlipped = false;
  private static SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private static SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkXPhoenixOdometryThread.getInstance().start();
    // Initialize Limelight camera streams for Elastic dashboard
    // if (Constants.currentMode == Mode.REAL) {
    //   setupLimelightStreams();
    // }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      isFlipped = true;
    }
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      if (!hasStoppedModules) {
        for (var module : modules) {
          module.stop();
        }
        hasStoppedModules = true;
      }
    } else {
      hasStoppedModules = false;
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      field.setRobotPose(getPose());
      SmartDashboard.putData("Field", field);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    if (Constants.currentMode == Mode.REAL) {
      LimelightHelpers.SetRobotOrientation(
          DriveConstants.kLimelightBackLeftName,
          gyroIO.getYaw().getDegrees(),
          0,
          gyroIO.getRoll().getDegrees(),
          0,
          gyroIO.getPitch().getDegrees(),
          0);

      LimelightHelpers.SetRobotOrientation(
          DriveConstants.kLimelightBackRightName,
          gyroIO.getYaw().getDegrees(),
          0,
          gyroIO.getRoll().getDegrees(),
          0,
          gyroIO.getPitch().getDegrees(),
          0);

      LimelightHelpers.SetRobotOrientation(
          DriveConstants.kLimelightFrontName,
          gyroIO.getYaw().getDegrees(),
          0,
          gyroIO.getRoll().getDegrees(),
          0,
          gyroIO.getPitch().getDegrees(),
          0);

      PoseEstimate bLMT2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightBackLeftName);
      PoseEstimate bRMT2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightBackRightName);
      PoseEstimate FLMT2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightFrontName);
      if (checkPose(bLMT2)) {
        updatePoseWithStdDev(bLMT2);
      }
      if (checkPose(bRMT2)) {
        updatePoseWithStdDev(bRMT2);
      }
      if (checkPose(FLMT2)) {
        updatePoseWithStdDev(FLMT2);
      }
    }
  }

  private void updatePoseWithStdDev(PoseEstimate estimate) {
    double avgDistance = estimate.avgTagDist;
    double xyStdDev =
        DriveConstants.xyStdDevCoefficient
            * Math.pow(avgDistance, 2.0)
            / estimate.tagCount
            * DriveConstants.stdDevFactor
            * (DriverStation.isAutonomous() ? DriveConstants.autoStdDevScale : 1.0);

    double thetaStdDev =
        useVisionRotation
            ? thetaStdDevCoefficient
                * Math.pow(avgDistance, 2.0)
                / estimate.tagCount
                * stdDevFactor
                * (DriverStation.isAutonomous() ? DriveConstants.autoStdDevScale : 1.0)
            : Double.POSITIVE_INFINITY;

    poseEstimator.addVisionMeasurement(
        estimate.pose, estimate.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    if (driveMode == ShooterStatus.SHOOT) {
      speeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * 0.2,
              speeds.vyMetersPerSecond * 0.2,
              speeds.omegaRadiansPerSecond * 0.5);
    } else if (driveMode == ShooterStatus.PASSING) {
      speeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * 0.8,
              speeds.vyMetersPerSecond * 0.8,
              speeds.omegaRadiansPerSecond * 1.0);
    } else if (driveMode == ShooterStatus.INTAKE) {
      speeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * 0.5,
              speeds.vyMetersPerSecond * 0.5,
              speeds.omegaRadiansPerSecond * 0.75);
    } else {
      speeds.times(1);
    }

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);
    robotChassisSpeeds = discreteSpeeds;

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveGyroRotation", rawGyroRotation);
  }

  public ChassisSpeeds getDriveSpeeds() {
    return robotChassisSpeeds;
  }

  public ChassisSpeeds getDriveSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()),
        poseEstimator.getEstimatedPosition().getRotation());
  }

  public double getPitch() {
    return gyroIO.getRoll().getDegrees();
  }

  public void runVelocityFieldRelative(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(Rotation2d.k180deg) : getRotation());
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    // resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void zeroHeading() {
    gyroIO.resetGyro();
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return gyroIO.getYaw();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return 2 * Math.PI;
  }
  // reset poses for auto
  public Command resetPoseWithLLS() {
    return runOnce(
        () -> {
          PoseEstimate leftLLMT2 =
              LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightBackLeftName);
          PoseEstimate rightLLMT2 =
              LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightBackRightName);
          PoseEstimate frontLLMT2 =
              LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightFrontName);
          PoseEstimate[] poseList = {leftLLMT2, rightLLMT2, frontLLMT2};
          PoseEstimate bestPose = null;
          double bestScore = Double.POSITIVE_INFINITY;
          for (PoseEstimate e : poseList) {
            if (e.tagCount > 0) {
              double eScore = e.avgTagDist + 1.0 / e.tagCount;
              if (eScore < bestScore) {
                bestScore = eScore;
                bestPose = e;
              }
            }
          }
          if (bestPose != null) {
            resetOdometry(bestPose.pose);
          }
        });
  }

  private boolean checkPose(PoseEstimate estimate) {
    if (estimate == null) {
      return false;
    }

    if (isEstimateZero(estimate)) {
      return false;
    }

    if (estimate.pose.getX() <= 0 || estimate.pose.getX() > 16.53) {
      return false;
    }

    if (estimate.pose.getY() <= 0 || estimate.pose.getY() > 8) {
      return false;
    }

    if (estimate.tagCount <= 0) {
      return false;
    }

    if (Math.abs(gyroIO.getRate()) > 720) {
      return false;
    }

    return true;
  }

  public void setDriveState(ShooterStatus mode) {
    driveMode = mode;
  }

  public ShooterStatus getDriveState() {
    return driveMode;
  }

  private boolean isEstimateZero(PoseEstimate estimate) {
    return estimate.pose.equals(new Pose2d());
  }

  public double getRoll() {
    return gyroIO.getRoll().getDegrees();
  }

  /**
   * Sets up Limelight camera streams for the Elastic dashboard. This publishes the camera feeds to
   * NetworkTables so they can be viewed in Elastic.
   */
  private void setupLimelightStreams() {
    try {
      // Back Left Limelight
      limelightBackLeft =
          new HttpCamera(
              kLimelightBackLeftName,
              "http://" + kLimelightBackLeftName + ".local:5800/stream.mjpg");
      CameraServer.startAutomaticCapture(limelightBackLeft);

      // Back Right Limelight
      limelightBackRight =
          new HttpCamera(
              kLimelightBackRightName,
              "http://" + kLimelightBackRightName + ".local:5800/stream.mjpg");
      CameraServer.startAutomaticCapture(limelightBackRight);

      // Front Limelight
      limelightFront =
          new HttpCamera(
              kLimelightFrontName, "http://" + kLimelightFrontName + ".local:5800/stream.mjpg");
      CameraServer.startAutomaticCapture(limelightFront);

      Logger.recordOutput("Drive/LimelightStreamsInitialized", true);
    } catch (Exception e) {
      Logger.recordOutput("Drive/LimelightStreamError", e.getMessage());
      System.err.println("Failed to initialize Limelight streams: " + e.getMessage());
    }
  }
}
