package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem with turret, hood, and dual flywheels. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Creates a new Shooter subsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /** Sets the flywheel velocity in RPM. */
  public void setFlywheelVelocity(double rpm) {
    io.setFlywheelVelocity(rpm, rpm);
  }

  /** Sets different velocities for left and right flywheels. */
  public void setFlywheelVelocity(double leftRPM, double rightRPM) {
    io.setFlywheelVelocity(leftRPM, rightRPM);
  }

  /** Sets the turret angle in degrees. */
  public void setTurretAngle(double degrees) {
    double clampedAngle =
        MathUtil.clamp(degrees, ShooterConstants.kTurretMinAngle, ShooterConstants.kTurretMaxAngle);
    io.setTurretPosition(clampedAngle);
  }

  /** Sets the hood angle in degrees. */
  public void setHoodAngle(double degrees) {
    double clampedAngle =
        MathUtil.clamp(degrees, ShooterConstants.kHoodMinAngle, ShooterConstants.kHoodMaxAngle);
    io.setHoodPosition(clampedAngle);
  }

  /** Returns true if the shooter is at the target velocity. */
  public boolean atTargetVelocity(double targetRPM) {
    double averageVelocity = (inputs.leftShooterVelocityRPM + inputs.rightShooterVelocityRPM) / 2.0;
    return Math.abs(averageVelocity - targetRPM) < ShooterConstants.kShooterToleranceRPM;
  }

  /** Returns true if the turret is at the target angle. */
  public boolean turretAtTarget(double targetDegrees) {
    return Math.abs(inputs.turretPositionDeg - targetDegrees)
        < ShooterConstants.kTurretToleranceDegrees;
  }

  /** Returns true if the hood is at the target angle. */
  public boolean hoodAtTarget(double targetDegrees) {
    return Math.abs(inputs.hoodPositionDeg - targetDegrees)
        < ShooterConstants.kHoodToleranceDegrees;
  }

  /** Stops all shooter motors. */
  public void stop() {
    io.stop();
  }

  /** Command to shoot at speaker. */
  public Command shootSpeakerCommand() {
    return run(() -> {
          setFlywheelVelocity(ShooterConstants.kSpeakerShootSpeed);
          setHoodAngle(45.0);
        })
        .withName("ShootSpeaker");
  }

  /** Command to shoot at amp. */
  public Command shootAmpCommand() {
    return run(() -> {
          setFlywheelVelocity(ShooterConstants.kAmpShootSpeed);
          setHoodAngle(30.0);
        })
        .withName("ShootAmp");
  }

  /** Command to idle the shooter. */
  public Command idleCommand() {
    return run(() -> setFlywheelVelocity(ShooterConstants.kIdleSpeed)).withName("Idle");
  }
}
