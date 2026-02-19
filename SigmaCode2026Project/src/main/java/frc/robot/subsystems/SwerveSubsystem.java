package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule[] modules;
  private final ADIS16470_IMU gyro;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final LimelightSubsystem limelight;

  // Field2d widget – shows robot position on the Shuffleboard field graphic
  private final Field2d field = new Field2d();

  // Module positions – distance between wheel centres in metres; adjust to your robot
  private static final double TRACK_WIDTH = 0.5842; // left ↔ right
  private static final double WHEEL_BASE = 0.5842; // front ↔ back

  // CAN IDs – adjust to match your wiring
  private static final int FRONT_LEFT_DRIVE_ID = 1;
  private static final int FRONT_LEFT_TURN_ID = 5;
  private static final int FRONT_RIGHT_DRIVE_ID = 2;
  private static final int FRONT_RIGHT_TURN_ID = 6;
  private static final int BACK_LEFT_DRIVE_ID = 3;
  private static final int BACK_LEFT_TURN_ID = 7;
  private static final int BACK_RIGHT_DRIVE_ID = 4;
  private static final int BACK_RIGHT_TURN_ID = 8;

  // Maximum wheel speed used for desaturation
  private static final double MAX_SPEED_MPS = 4.5;

  // Pose-estimator standard deviations [x (m), y (m), θ (rad)]
  private static final double[] STATE_STD_DEVS = {0.1, 0.1, 0.1};
  private static final double[] VISION_STD_DEVS = {0.5, 0.5, 0.5};

  public SwerveSubsystem(LimelightSubsystem limelight) {
    this.limelight = limelight;

    modules =
        new SwerveModule[] {
          new SwerveModule(0, FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID),
          new SwerveModule(1, FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID),
          new SwerveModule(2, BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID),
          new SwerveModule(3, BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID)
        };

    gyro = new ADIS16470_IMU();

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // Front Left
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Front Right
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // Back Left
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0) // Back Right
            );

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(STATE_STD_DEVS[0], STATE_STD_DEVS[1], STATE_STD_DEVS[2]),
            VecBuilder.fill(VISION_STD_DEVS[0], VISION_STD_DEVS[1], VISION_STD_DEVS[2]));

    SmartDashboard.putData("Field", field);
  }

  // ── Periodic ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // 1. Odometry update (wheel encoders + gyro)
    poseEstimator.update(getRotation2d(), getModulePositions());

    // 2. Fuse vision measurement when the Limelight has a reliable fix
    if (limelight.hasValidMeasurement()) {
      poseEstimator.addVisionMeasurement(limelight.getRobotPose(), limelight.getTimestampSeconds());
    }

    // 3. Push telemetry to dashboard
    Pose2d pose = getPose();
    field.setRobotPose(pose);
    SmartDashboard.putNumber("Robot X (m)", pose.getX());
    SmartDashboard.putNumber("Robot Y (m)", pose.getY());
    SmartDashboard.putNumber("Robot Heading°", pose.getRotation().getDegrees());
  }

  // ── Drive ─────────────────────────────────────────────────────────────────

  /**
   * Drive the robot.
   *
   * @param xSpeed forward (+) / back (−) speed in m/s
   * @param ySpeed left (+) / right (−) speed in m/s
   * @param rot CCW (+) / CW (−) angular speed in rad/s
   * @param fieldRelative {@code true} for field-relative, {@code false} for robot-relative
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_SPEED_MPS);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(moduleStates[i]);
    }
  }

  // ── Pose ──────────────────────────────────────────────────────────────────

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  // ── Gyro ──────────────────────────────────────────────────────────────────

  public Rotation2d getRotation2d() {
    // Negate because ADIS16470 reports CW as positive; WPILib expects CCW positive.
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public void resetGyro() {
    gyro.reset();
  }

  // ── Module helpers ────────────────────────────────────────────────────────

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }
}
