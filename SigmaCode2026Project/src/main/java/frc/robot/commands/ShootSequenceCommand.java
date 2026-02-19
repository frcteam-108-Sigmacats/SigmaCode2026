package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.SpinDexer;

/**
 * Complete shooting sequence that spins up flywheels, waits for target velocity, then kicks the
 * game piece through the shooter.
 */
public class ShootSequenceCommand extends Command {
  private final Shooter shooter;
  private final SpinDexer spinDexer;
  private final double targetRPM;
  private final double hoodAngle;

  private boolean flywheelsReady = false;
  private boolean kicked = false;
  private double kickStartTime = 0;

  /**
   * Creates a new ShootSequenceCommand.
   *
   * @param shooter The shooter subsystem
   * @param spinDexer The spin dexer subsystem
   * @param targetRPM Target flywheel velocity in RPM
   * @param hoodAngle Target hood angle in degrees
   */
  public ShootSequenceCommand(
      Shooter shooter, SpinDexer spinDexer, double targetRPM, double hoodAngle) {
    this.shooter = shooter;
    this.spinDexer = spinDexer;
    this.targetRPM = targetRPM;
    this.hoodAngle = hoodAngle;

    addRequirements(shooter, spinDexer);
  }

  /** Creates a shoot sequence for the speaker target. */
  public static ShootSequenceCommand forSpeaker(Shooter shooter, SpinDexer spinDexer) {
    return new ShootSequenceCommand(shooter, spinDexer, ShooterConstants.kSpeakerShootSpeed, 45.0);
  }

  /** Creates a shoot sequence for the amp target. */
  public static ShootSequenceCommand forAmp(Shooter shooter, SpinDexer spinDexer) {
    return new ShootSequenceCommand(shooter, spinDexer, ShooterConstants.kAmpShootSpeed, 30.0);
  }

  @Override
  public void initialize() {
    flywheelsReady = false;
    kicked = false;
    kickStartTime = 0;
  }

  @Override
  public void execute() {
    // Always run shooter at target velocity
    shooter.setFlywheelVelocity(targetRPM);
    shooter.setHoodAngle(hoodAngle);

    // Check if flywheels are at target
    if (!flywheelsReady && shooter.atTargetVelocity(targetRPM)) {
      flywheelsReady = true;
    }

    // Once ready, kick the game piece
    if (flywheelsReady && !kicked) {
      spinDexer.runKicker(12.0 * 0.8);
      kickStartTime = System.currentTimeMillis();
      kicked = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    spinDexer.stop();

    if (!interrupted) {
      // Keep shooter at idle speed if completed successfully
      shooter.setFlywheelVelocity(ShooterConstants.kIdleSpeed);
    } else {
      shooter.stop();
    }
  }

  @Override
  public boolean isFinished() {
    // Finish 0.5 seconds after kicking
    return kicked && (System.currentTimeMillis() - kickStartTime) > 500;
  }
}
