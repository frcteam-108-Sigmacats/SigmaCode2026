package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.Shooter;
import java.util.function.DoubleSupplier;

/**
 * Static factory class for Turret commands.
 *
 * <p>Naming convention mirrors {@link DriveCommands}: every method returns a {@link Command} and is
 * named for what it does, not how it does it.
 *
 * <p>Basic commands:
 *
 * <ul>
 *   <li>{@link #stopAll} – coast everything to zero immediately
 *   <li>{@link #manualTurret} – operator jogs the turret with a joystick axis
 *   <li>{@link #spinUpShooter} – run wheels to a target surface speed (m/s)
 *   <li>{@link #idleShooter} – coast wheels (does NOT hold speed)
 *   <li>{@link #setHoodAngle} – servo hood to a fixed angle and finish when there
 *   <li>{@link #prepareToShoot} – spin wheels + servo hood together, finishes when both ready
 * </ul>
 */
public class TurretCommands {

  private static final double TURRET_DEADBAND = 0.08;

  /** Maximum turret jog voltage when an axis is fully deflected. */
  private static final double TURRET_MAX_VOLTS = 4.0;

  private TurretCommands() {}

  // ── Basic stop ────────────────────────────────────────────────────────────

  /**
   * Immediately cut power to every motor on the turret subsystem and coast to a stop. This command
   * runs forever so it is suitable as a default command.
   */
  public static Command stopAll(Shooter turret) {
    return Commands.run(turret::stopAll, turret).withName("Turret.StopAll");
  }

  // ── Manual joystick control ───────────────────────────────────────────────

  /**
   * Jog the turret rotation ring open-loop from a joystick axis.
   *
   * <p>Applies a deadband and squares the input for finer control near centre. Safe to use as the
   * turret default command because it emits 0 V when the axis is inside the deadband.
   *
   * @param turret the turret subsystem
   * @param axisSupplier raw joystick axis in [-1, +1]
   */
  public static Command manualTurret(Shooter turret, DoubleSupplier axisSupplier) {
    return Commands.run(
            () -> {
              double raw = MathUtil.applyDeadband(axisSupplier.getAsDouble(), TURRET_DEADBAND);
              double volts = Math.copySign(raw * raw, raw) * TURRET_MAX_VOLTS;
              turret.setTurretOpenLoop(volts);
            },
            turret)
        .finallyDo(turret::stopTurret)
        .withName("Turret.ManualTurret");
  }

  // ── Shooter wheels ────────────────────────────────────────────────────────

  /**
   * Spin the shooter wheels to {@code targetMPS} surface speed and keep running until interrupted.
   * The command does NOT finish on its own; use {@link Command#until} or bind it to a button to
   * interrupt it.
   *
   * <p>Example – hold A to spin up:
   *
   * <pre>
   *   controller.a().whileTrue(TurretCommands.spinUpShooter(turret, 15.0));
   * </pre>
   *
   * @param turret the turret subsystem
   * @param targetMPS desired surface speed in metres per second
   */
  public static Command spinUpShooter(Shooter turret, double targetMPS) {
    return Commands.run(() -> turret.setShooterSpeed(targetMPS), turret)
        .finallyDo(turret::stopShooter)
        .withName("Turret.SpinUpShooter(" + targetMPS + " mps)");
  }

  /**
   * Coast the shooter wheels to a stop by cutting motor output. Finishes immediately (instant
   * command).
   */
  public static Command idleShooter(Shooter turret) {
    return Commands.runOnce(turret::stopShooter, turret).withName("Turret.IdleShooter");
  }

  // ── Hood positioning ──────────────────────────────────────────────────────

  /**
   * Servo the hood to a fixed angle and finish once the hood is within tolerance. Times out after 2
   * seconds to avoid hanging if the hood is stuck.
   *
   * @param turret the turret subsystem
   * @param angleDeg target elevation in degrees [0, 90]
   */
  public static Command setHoodAngle(Shooter turret, double angleDeg) {
    return Commands.run(() -> turret.setHoodAngle(angleDeg), turret)
        .until(turret::isHoodAtPosition)
        .withTimeout(2.0)
        .withName("Turret.SetHoodAngle(" + angleDeg + " deg)");
  }

  // ── Composite ─────────────────────────────────────────────────────────────

  /**
   * Simultaneously spin the shooter wheels and servo the hood to the requested positions. Finishes
   * once both the wheels are at speed <em>and</em> the hood is at the target angle. Times out after
   * 3 seconds.
   *
   * <p>Typical use: run this command before firing so you know the robot is ready.
   *
   * <pre>
   *   controller.rightBumper()
   *       .whileTrue(TurretCommands.prepareToShoot(turret, 15.0, 60.0)
   *           .andThen(/* fire command *\/));
   * </pre>
   *
   * @param turret the turret subsystem
   * @param targetMPS shooter surface speed in metres per second
   * @param hoodDeg hood elevation in degrees
   */
  public static Command prepareToShoot(Shooter turret, double targetMPS, double hoodDeg) {
    return Commands.run(
            () -> {
              turret.setShooterSpeed(targetMPS);
              turret.setHoodAngle(hoodDeg);
            },
            turret)
        .until(() -> turret.isShooterAtSpeed() && turret.isHoodAtPosition())
        .withTimeout(3.0)
        .withName("Turret.PrepareToShoot");
  }
}
