package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Velocity tolerance for "at speed" check (rad/s)
  private static final double SHOOTER_VELOCITY_TOLERANCE_RAD_PER_SEC = 10.0;
  // Angle tolerance for "on target" check (rad)
  private static final double TURRET_ANGLE_TOLERANCE_RAD = Math.toRadians(1.5);
  // Hood position tolerance (deg)
  private static final double HOOD_TOLERANCE_DEG = 1.5;

  private double desiredShooterVelocityRadPerSec = 0.0;
  private double desiredTurretAngleRad = 0.0;
  private double desiredHoodAngleDeg = 0.0;

  private boolean blueAlliance;

  public Shooter(ShooterIO io) {
    this.io = io;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      blueAlliance = true;
    } else {
      blueAlliance = false;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Derived outputs for dashboard / logging
    Logger.recordOutput("Shooter/OnTarget", isOnTarget());
    Logger.recordOutput("Shooter/ShooterAtSpeed", isShooterAtSpeed());
    Logger.recordOutput("Shooter/HoodAtPosition", isHoodAtPosition());
    Logger.recordOutput("Shooter/ReadyToShoot", isReadyToShoot());
    Logger.recordOutput(
        "Turret/ShooterSurfaceMPS",
        (inputs.shooterLeftVelocityRPM / 60)
            * ShooterConstants.shooterWheelRadiusMeters
            * 2
            * Math.PI);
  }

  // ── Turret Rotation API ───────────────────────────────────────────────────

  /** Command the turret to a target angle */
  public void setTurretAngle(Rotation2d angle) {
    desiredTurretAngleRad = angle.getRadians();
    io.setTurretPosition(desiredTurretAngleRad);
  }

  /** Run the turret rotation motor open-loop (volts). */
  public void setTurretOpenLoop(double volts) {
    io.setTurretOpenLoop(volts);
  }

  /** Stop the turret rotation motor. */
  public void stopTurret() {
    io.setTurretOpenLoop(0.0);
  }

  /** Current turret angle as a {@link Rotation2d}. */
  public Rotation2d getTurretAngle() {
    return inputs.turretPosition;
  }

  /** Whether the turret is within {@value TURRET_ANGLE_TOLERANCE_RAD} rad of its target. */
  public boolean isOnTarget() {
    return Math.abs(inputs.turretPosition.getRadians() - desiredTurretAngleRad)
        < TURRET_ANGLE_TOLERANCE_RAD;
  }

  // ── Shooter Flywheel API ──────────────────────────────────────────────────

  /**
   * Spin both shooter wheels to the requested surface speed in meters per second.
   *
   * @param distance The distance that the robot is from the target
   * @param fullSpeed indicates whether we should go at full speed or not
   */
  public void setShooterSpeed(double distance, boolean fullSpeed) {
    double rpm =
        getInterpolated(Double.valueOf(distance), ShooterConstants.ShooterStates.shooterRPMMap);
    if (fullSpeed) {
      io.setShooterVelocity(rpm);
    } else {
      io.setShooterVelocity(2500);
    }
  }

  /** Run shooter wheels open-loop (volts). */
  public void setShooterOpenLoop(double volts) {
    io.setShooterOpenLoop(volts);
  }

  /** Coast both wheels to a stop. */
  public void stopShooter() {
    io.setShooterOpenLoop(0.0);
  }

  public Double getInterpolated(Double key, TreeMap<Double, Double> dataMap) {
    if (dataMap.isEmpty()) {
      return null;
    }
    Double floorKey = dataMap.floorKey(key);
    Double ceilingKey = dataMap.ceilingKey(key);

    if (floorKey == null) {
      return dataMap.get(ceilingKey);
    }
    if (ceilingKey == null) {
      return dataMap.get(floorKey);
    }

    if (floorKey.equals(ceilingKey)) {
      return dataMap.get(key);
    }

    Double y1 = dataMap.get(floorKey);
    Double y2 = dataMap.get(ceilingKey);

    double fraction = (key - floorKey) / (ceilingKey - floorKey);

    return y1 + (y2 - y1) * fraction;
  }

  /** Average shooter wheel velocity in rad/s. */
  public double getShooterVelocityRPM() {
    return (inputs.shooterLeftVelocityRPM + inputs.shooterRightVelocityRPM) / 2.0;
  }

  /** Whether both wheels are within tolerance of the current setpoint. */
  public boolean isShooterAtSpeed() {
    if (desiredShooterVelocityRadPerSec == 0.0) return false;
    return Math.abs(inputs.shooterLeftVelocityRPM - desiredShooterVelocityRadPerSec)
            < SHOOTER_VELOCITY_TOLERANCE_RAD_PER_SEC
        && Math.abs(inputs.shooterRightVelocityRPM - desiredShooterVelocityRadPerSec)
            < SHOOTER_VELOCITY_TOLERANCE_RAD_PER_SEC;
  }

  // ── Hood API ──────────────────────────────────────────────────────────────

  /**
   * Command the hood to a target elevation angle in degrees.
   *
   * @param angleDeg angle in [0°, 27°]
   */
  public void setHoodAngle(double distance) {
    double angle =
        getInterpolated(
            Double.valueOf(distance), ShooterConstants.ShooterStates.shooterHoodAngleMap);
    io.setHoodPosition(angle); // auto
  }

  /** Run hood motor open-loop (volts). */
  public void setHoodOpenLoop(double volts) {
    io.setHoodOpenLoop(volts);
  }

  /** Stop the hood motor. */
  public void stopHood() {
    io.setHoodOpenLoop(0.0);
  }

  /** Current hood elevation in degrees. */
  public double getHoodAngleDeg() {
    return inputs.hoodPositionDeg;
  }

  /** Whether the hood is within {@value HOOD_TOLERANCE_DEG}° of its target. */
  public boolean isHoodAtPosition() {
    return Math.abs(inputs.hoodPositionDeg - desiredHoodAngleDeg) < HOOD_TOLERANCE_DEG;
  }

  /**
   * Figures out where the robot resides in the field to pinpoint where we should be shooting at the
   * time
   *
   * @param swerveDrive The subsystem that holds the robots position
   * @return target pose
   */
  public Pose2d getTargetPose(Drive swerveDrive) {
    // Checks which Alliance color we are on
    if (blueAlliance) {
      // Checks if the robot position is outside the alliance zone
      if (swerveDrive.getPose().getX() > ShooterConstants.blueHubPose.getX()) {
        // Checks if the robot Y pose is closer to the depot or outpost side
        if (swerveDrive.getPose().getY() > ShooterConstants.blueHubPose.getY()) {
          return ShooterConstants.blueDepotPose;
        } else {
          return ShooterConstants.blueStationPose;
        }
      } else {
        return ShooterConstants.blueHubPose;
      }
    } else {
      // Checks if the robot position is outside the alliance zone
      if (swerveDrive.getPose().getX() < ShooterConstants.redHubPose.getX()) {
        // Checks if the robot Y pose is closer to the depot or outpost side
        if (swerveDrive.getPose().getY() > ShooterConstants.redHubPose.getY()) {
          return ShooterConstants.redStationPose;
        } else {
          return ShooterConstants.redDepotPose;
        }
      } else {
        return ShooterConstants.redHubPose;
      }
    }
  }
  /**
   * Calculates the futuristic target Pose by calculating time of flight with chassis speeds to
   * offset target pose
   *
   * @param targetPose The pose we want to shoot our fuel at
   * @param swerveDrive The subsystem that holds the robot position and chassis speeds
   * @return The pose the robot should be aiming at
   */
  public Translation2d getAimPoint(Pose2d targetPose, Drive swerveDrive) {
    Translation2d robotPos = swerveDrive.getPose().getTranslation();
    ChassisSpeeds fieldSpeeds = swerveDrive.getDriveSpeedsFieldRelative();

    Translation2d aimPoint = targetPose.getTranslation();

    Translation2d turretOffset = ShooterConstants.turretOffset.rotateBy(swerveDrive.getRotation());
    double flightOfTime = 0;
    double distance = 0;
    // Calculates the correct aim point a few times for better accuracy
    for (int i = 0; i < 5; i++) {
      distance = targetPose.getTranslation().minus(robotPos).getNorm();

      double RPM =
          getInterpolated(Double.valueOf(distance), ShooterConstants.ShooterStates.shooterRPMMap);
      double hoodAngle =
          getInterpolated(
              Double.valueOf(distance), ShooterConstants.ShooterStates.shooterHoodAngleMap);

      double exitBallVelX =
          (RPM * ShooterConstants.ballExitVelocityConversion)
              * Math.cos(Math.toRadians(ShooterConstants.hoodStartAngle + hoodAngle));
      flightOfTime =
          (distance / exitBallVelX)
              * 1.46; // Had to multiply by a factor of 1.46 to get more accurate flight of time
      // Calculates the X and Y velocity of the turret to deal with rotational velocity compensation
      Translation2d tangentialTurretVel =
          new Translation2d(
              -turretOffset.getY() * fieldSpeeds.omegaRadiansPerSecond,
              turretOffset.getX() * fieldSpeeds.omegaRadiansPerSecond);
      // Offsets the target pose based on linear and rotational velocity compensation
      aimPoint =
          new Translation2d(
              targetPose.getX()
                  - (fieldSpeeds.vxMetersPerSecond + tangentialTurretVel.getX()) * flightOfTime,
              targetPose.getY()
                  - (fieldSpeeds.vyMetersPerSecond + tangentialTurretVel.getY()) * flightOfTime);
    }
    // Records basic information needed for SOTM
    Logger.recordOutput("Shooter/Distance", distance);
    Logger.recordOutput("Shooter/FieldSpeeds", fieldSpeeds);
    Logger.recordOutput("/Shooter/TimeOfFlight", flightOfTime);
    return aimPoint;
  }

  // ── Composite ─────────────────────────────────────────────────────────────

  /**
   * Returns {@code true} when the turret is on target, shooter wheels are at speed, and the hood is
   * at the desired elevation – i.e. the robot is ready to fire.
   */
  public boolean isReadyToShoot() {
    return isOnTarget() && isShooterAtSpeed() && isHoodAtPosition();
  }

  /** Stop all motors immediately. */
  public void stopAll() {
    stopTurret();
    stopShooter();
    stopHood();
  }
}
