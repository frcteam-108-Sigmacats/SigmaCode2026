package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Default teleop drive command for a swerve drivetrain.
 *
 * <p>Takes joystick inputs from an Xbox One controller via lambda suppliers so that this command
 * stays decoupled from the specific controller object.
 */
public class DefaultDriveCommand extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier xSupplier; // forward / back
  private final DoubleSupplier ySupplier; // strafe left / right
  private final DoubleSupplier rotSupplier; // rotation (CCW positive)
  private final BooleanSupplier fieldRelativeSupplier;

  /** Maximum linear speed (m/s) – tune to your robot. */
  private static final double MAX_SPEED_MPS = 4.5;

  /** Maximum angular speed (rad/s) – tune to your robot. */
  private static final double MAX_ANGULAR_SPEED = Math.PI;

  /** Joystick deadband – eliminates stick drift. */
  private static final double DEADBAND = 0.1;

  /**
   * @param swerve the swerve subsystem
   * @param xSupplier forward (+) / back (−) supplier [−1, 1]
   * @param ySupplier left (+) / right (−) supplier [−1, 1]
   * @param rotSupplier CCW (+) / CW (−) supplier [−1, 1]
   * @param fieldRelativeSupplier true → field-relative, false → robot-relative
   */
  public DefaultDriveCommand(
      SwerveSubsystem swerve,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      BooleanSupplier fieldRelativeSupplier) {

    this.swerve = swerve;

    // FIX: Guard against null suppliers that were being passed from the original
    // RobotContainer. Null suppliers caused a NullPointerException on execute().
    this.xSupplier = (xSupplier != null) ? xSupplier : () -> 0.0;
    this.ySupplier = (ySupplier != null) ? ySupplier : () -> 0.0;
    this.rotSupplier = (rotSupplier != null) ? rotSupplier : () -> 0.0;
    this.fieldRelativeSupplier =
        (fieldRelativeSupplier != null) ? fieldRelativeSupplier : () -> true;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double xSpeed = applyDeadband(xSupplier.getAsDouble()) * MAX_SPEED_MPS;
    double ySpeed = applyDeadband(ySupplier.getAsDouble()) * MAX_SPEED_MPS;
    double rotSpeed = applyDeadband(rotSupplier.getAsDouble()) * MAX_ANGULAR_SPEED;
    boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();

    swerve.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  /**
   * Applies a deadband and rescales the output so there is no discontinuity at the deadband edge
   * (i.e. the output goes smoothly from 0 → 1 as the stick moves from the deadband boundary to full
   * deflection).
   */
  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, DEADBAND);
  }
}
