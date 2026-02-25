package frc.robot.commands;
//─────────────────────────────────────────────────────────────
//Very low chance this works it took awhile but this its it
//─────────────────────────────────────────────────────────────
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SmartAimCommand extends Command {

  private final Shooter shooter;
  private final Supplier<Pose2d> robotPoseSupplier;

  private Pose2d hubPose;
  private Pose2d depotPose;
  private Pose2d humanStationPose;
  private double allianceZoneXThreshold;
  private boolean isBlueAlliance;

  private static final double WHEEL_DIAMETER_M = 2.0 * ShooterConstants.shooterWheelRadiusMeters;

  public SmartAimCommand(Shooter shooter, Supplier<Pose2d> robotPoseSupplier) {
    this.shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();

    isBlueAlliance = allianceOptional.isEmpty() || allianceOptional.get() == Alliance.Blue;

    if (isBlueAlliance) {
      hubPose = ShooterConstants.BLUE_HUB_POSE;
      depotPose = ShooterConstants.BLUE_DEPOT_POSE;
      humanStationPose = ShooterConstants.BLUE_HUMAN_STATION_POSE;
      allianceZoneXThreshold = ShooterConstants.BLUE_ALLIANCE_ZONE_X_THRESHOLD;
    } else {
      hubPose = ShooterConstants.RED_HUB_POSE;
      depotPose = ShooterConstants.RED_DEPOT_POSE;
      humanStationPose = ShooterConstants.RED_HUMAN_STATION_POSE;
      allianceZoneXThreshold = ShooterConstants.RED_ALLIANCE_ZONE_X_THRESHOLD;
    }

    Logger.recordOutput("SmartAim/Alliance", isBlueAlliance ? "Blue" : "Red");
  }

  @Override
  public void execute() {
    Pose2d robotPose = robotPoseSupplier.get();

    Pose2d targetPose = determineTargetPose(robotPose);

    Translation2d robotToTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
    double distanceToTarget = robotToTarget.getNorm();

    double fieldRelativeAngleRad = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double robotRelativeAngleRad = fieldRelativeAngleRad - robotHeadingRad;

    robotRelativeAngleRad = normalizeAngle(robotRelativeAngleRad);

    double rpm = interpolate(distanceToTarget, ShooterConstants.ShooterStates.shooterRPMMap);
    double hoodDeg = interpolate(distanceToTarget, ShooterConstants.ShooterStates.shooterHoodAngleMap);
    double surfaceMps = (rpm / 60.0) * Math.PI * WHEEL_DIAMETER_M;

    shooter.setTurretAngle(new Rotation2d(robotRelativeAngleRad));
    shooter.setShooterSpeed(surfaceMps);
    shooter.setHoodAngle(hoodDeg);

    Logger.recordOutput("SmartAim/RobotX", robotPose.getX());
    Logger.recordOutput("SmartAim/RobotY", robotPose.getY());
    Logger.recordOutput("SmartAim/TargetPoseX", targetPose.getX());
    Logger.recordOutput("SmartAim/TargetPoseY", targetPose.getY());
    Logger.recordOutput("SmartAim/DistanceToTarget", distanceToTarget);
    Logger.recordOutput("SmartAim/TurretAngleDeg", Math.toDegrees(robotRelativeAngleRad));
    Logger.recordOutput("SmartAim/ShooterRPM", rpm);
    Logger.recordOutput("SmartAim/HoodAngleDeg", hoodDeg);
    Logger.recordOutput("SmartAim/ReadyToShoot", shooter.isReadyToShoot());
  }

  private Pose2d determineTargetPose(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();

    boolean inAllianceZone;

    if (isBlueAlliance) {
      inAllianceZone = robotX <= allianceZoneXThreshold;
    } else {
      inAllianceZone = robotX >= allianceZoneXThreshold;
    }

    Pose2d selectedPose;

    if (inAllianceZone) {
      selectedPose = hubPose;
      Logger.recordOutput("SmartAim/TargetLocation", "Hub");
    } else {
      if (robotY < ShooterConstants.FIELD_CENTER_Y) {
        selectedPose = depotPose;
        Logger.recordOutput("SmartAim/TargetLocation", "Depot");
      } else {
        selectedPose = humanStationPose;
        Logger.recordOutput("SmartAim/TargetLocation", "HumanStation");
      }
    }

    return selectedPose;
  }

  private double normalizeAngle(double angleRad) {
    double normalized = angleRad;
    while (normalized > Math.PI) {
      normalized -= 2.0 * Math.PI;
    }
    while (normalized < -Math.PI) {
      normalized += 2.0 * Math.PI;
    }
    return normalized;
  }

  private static double interpolate(double key, TreeMap<Double, Double> dataMap) {
    if (dataMap.isEmpty()) return 0.0;

    Double lo = dataMap.floorKey(key);
    Double hi = dataMap.ceilingKey(key);

    if (lo == null) return dataMap.get(hi);
    if (hi == null) return dataMap.get(lo);
    if (lo.equals(hi)) return dataMap.get(lo);

    double y1 = dataMap.get(lo);
    double y2 = dataMap.get(hi);
    double fraction = (key - lo) / (hi - lo);
    return y1 + (y2 - y1) * fraction;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
