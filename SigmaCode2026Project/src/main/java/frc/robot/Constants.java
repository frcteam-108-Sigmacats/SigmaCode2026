package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double kRotationDeadband = 0.1;
  }

  public static final class DriveConstants {
    // CAN IDs for drive motors (Kraken X60)
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontRightDriveMotorId = 3;
    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackRightDriveMotorId = 7;

    // CAN IDs for steering motors (Neo 550)
    public static final int kFrontLeftSteerMotorId = 2;
    public static final int kFrontRightSteerMotorId = 4;
    public static final int kBackLeftSteerMotorId = 6;
    public static final int kBackRightSteerMotorId = 8;

    // CAN IDs for CANcoders (through bore encoders)
    public static final int kFrontLeftEncoderId = 9;
    public static final int kFrontRightEncoderId = 10;
    public static final int kBackLeftEncoderId = 11;
    public static final int kBackRightEncoderId = 12;

    // Pigeon 2 IMU CAN ID
    public static final int kPigeonId = 13;

    // Limelight names
    public static final String kLimelightFrontName = "limelight-front";
    public static final String kLimelightBackName = "limelight-back";
    public static final String kLimelightShooterName = "limelight-shooter";

    // Physical constants
    public static final double kTrackWidth = Units.inchesToMeters(24.0);
    public static final double kWheelBase = Units.inchesToMeters(24.0);
    public static final double kWheelDiameter = Units.inchesToMeters(4.0);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;

    // Swerve module positions (relative to center of robot)
    public static final Translation2d kFrontLeftLocation =
        new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d kFrontRightLocation =
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d kBackLeftLocation =
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d kBackRightLocation =
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);

    // Drive motor configuration
    public static final double kDriveGearRatio = 6.75; // L2 SDS Modules
    public static final double kSteerGearRatio = 12.8; // SDS Mk4i

    // Encoder offsets (set these during calibration)
    public static final Rotation2d kFrontLeftEncoderOffset = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kFrontRightEncoderOffset = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kBackLeftEncoderOffset = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kBackRightEncoderOffset = Rotation2d.fromDegrees(0.0);

    // Drive characteristics
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    // PID constants for steering
    public static final double kSteerP = 0.01;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.0;

    // PID constants for drive
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    // Current limits
    public static final int kDriveCurrentLimit = 80;
    public static final int kSteerCurrentLimit = 40;
  }

  public static final class IntakeConstants {
    // CAN IDs for intake motors
    public static final int kIntakeActuationMotorId = 14; // Neo 1.1 for actuation
    public static final int kIntakeRollerMotorId = 15; // Neo Vortex for roller

    // Motor configuration
    public static final boolean kActuationMotorInverted = false;
    public static final boolean kRollerMotorInverted = true;

    // Current limits
    public static final int kActuationCurrentLimit = 40;
    public static final int kRollerCurrentLimit = 60;

    // Speed constants
    public static final double kIntakeSpeed = 0.8;
    public static final double kOuttakeSpeed = -0.5;

    // Sensor configuration
    public static final int kBeamBreakChannel = 0; // DIO channel for beam break sensor
  }

  public static final class SpinDexerConstants {
    // CAN IDs for SpinDexer motors
    public static final int kSpinDexerMotorId = 16; // Neo 1.1 for indexer
    public static final int kKickerMotor1Id = 17; // Neo Vortex for kicker
    public static final int kKickerMotor2Id = 18; // Neo Vortex for kicker

    // Motor configuration
    public static final boolean kSpinDexerInverted = false;
    public static final boolean kKicker1Inverted = false;
    public static final boolean kKicker2Inverted = true; // Opposite of kicker 1

    // Current limits
    public static final int kSpinDexerCurrentLimit = 40;
    public static final int kKickerCurrentLimit = 60;

    // Speed constants
    public static final double kIndexSpeed = 0.3;
    public static final double kKickerSpeed = 0.8;

    // Positions (in rotations)
    public static final double kPosition1 = 0.0;
    public static final double kPosition2 = 0.333;
    public static final double kPosition3 = 0.667;

    // PID constants
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Sensor configuration
    public static final int kProximitySensor1 = 1; // DIO channel
    public static final int kProximitySensor2 = 2; // DIO channel
    public static final int kProximitySensor3 = 3; // DIO channel
  }

  public static final class ShooterConstants {
    // CAN IDs for shooter motors (Kraken X60)
    public static final int kLeftShooterMotorId = 19;
    public static final int kRightShooterMotorId = 20;

    // CAN ID for turret motor (Neo Vortex)
    public static final int kTurretMotorId = 21;

    // CAN ID for hood motor (Neo 550)
    public static final int kHoodMotorId = 22;

    // Absolute encoder for turret
    public static final int kTurretEncoderId = 23;

    // Motor configuration
    public static final boolean kLeftShooterInverted = false;
    public static final boolean kRightShooterInverted = true;
    public static final boolean kTurretInverted = false;
    public static final boolean kHoodInverted = false;

    // Current limits
    public static final int kShooterCurrentLimit = 80;
    public static final int kTurretCurrentLimit = 60;
    public static final int kHoodCurrentLimit = 30;

    // Flywheel speeds (RPM)
    public static final double kSpeakerShootSpeed = 4000;
    public static final double kAmpShootSpeed = 2000;
    public static final double kIdleSpeed = 500;

    // Turret limits (degrees)
    public static final double kTurretMinAngle = -90;
    public static final double kTurretMaxAngle = 90;
    public static final Rotation2d kTurretOffset = Rotation2d.fromDegrees(0.0);

    // Hood limits (degrees)
    public static final double kHoodMinAngle = 10;
    public static final double kHoodMaxAngle = 60;

    // PID constants for shooter flywheel
    public static final double kShooterP = 0.0001;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterFF = 0.00019;

    // PID constants for turret
    public static final double kTurretP = 0.01;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.0;

    // PID constants for hood
    public static final double kHoodP = 0.05;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.0;

    // Tolerances
    public static final double kShooterToleranceRPM = 50;
    public static final double kTurretToleranceDegrees = 1.0;
    public static final double kHoodToleranceDegrees = 0.5;
  }

  public static final class VisionConstants {
    // Camera mounting (adjust based on actual mounting)
    public static final double kCameraHeightMeters = Units.inchesToMeters(24.0);
    public static final double kCameraPitchRadians = Units.degreesToRadians(20.0);

    // AprilTag IDs for Rebuilt 2026
    public static final int kBlueSourceLeft = 1;
    public static final int kBlueSourceRight = 2;
    public static final int kRedSpeakerCenter = 3;
    public static final int kRedSpeakerOffset = 4;
    public static final int kRedAmp = 5;
    public static final int kBlueAmp = 6;
    public static final int kBlueSpeakerOffset = 7;
    public static final int kBlueSpeakerCenter = 8;
    public static final int kRedSourceRight = 9;
    public static final int kRedSourceLeft = 10;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    // PID constants for path following
    public static final double kPXController = 5.0;
    public static final double kPYController = 5.0;
    public static final double kPThetaController = 5.0;
  }

  public static final class SimConstants {
    // Simulation physics constants
    public static final double kSimLoopPeriod = 0.02; // 20ms
  }
}
