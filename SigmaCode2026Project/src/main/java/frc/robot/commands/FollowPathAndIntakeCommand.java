package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.SpinDexer;

/**
 * Follows a PathPlanner path while simultaneously running the intake. Useful for autonomous
 * routines where you want to pick up game pieces while driving to the next position.
 */
public class FollowPathAndIntakeCommand extends Command {
  private final Drive drive;
  private final Intake intake;
  private final SpinDexer spinDexer;
  private final String pathName;

  private Command parallelCommand;

  /**
   * Creates a new FollowPathAndIntakeCommand.
   *
   * @param drive The drive subsystem
   * @param intake The intake subsystem
   * @param spinDexer The spin dexer subsystem
   * @param pathName The name of the PathPlanner path to follow
   */
  public FollowPathAndIntakeCommand(
      Drive drive, Intake intake, SpinDexer spinDexer, String pathName) {

    this.drive = drive;
    this.intake = intake;
    this.spinDexer = spinDexer;
    this.pathName = pathName;

    addRequirements(drive, intake, spinDexer);
  }

  @Override
  public void initialize() {
    // Load the path
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    // Create path following command
    Command followPathCommand = AutoBuilder.followPath(path);

    // Create auto intake command
    Command autoIntakeCommand = new AutoIntakeCommand(intake, spinDexer);

    // Run both in parallel
    parallelCommand = Commands.parallel(followPathCommand, autoIntakeCommand);

    parallelCommand.initialize();
  }

  @Override
  public void execute() {
    if (parallelCommand != null) {
      parallelCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (parallelCommand != null) {
      parallelCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return parallelCommand != null && parallelCommand.isFinished();
  }

  /**
   * Factory method to create a race condition version where the command ends when either the path
   * finishes OR a game piece is intaked.
   */
  public static Command raceVersion(
      Drive drive, Intake intake, SpinDexer spinDexer, String pathName) {

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return Commands.race(AutoBuilder.followPath(path), new AutoIntakeCommand(intake, spinDexer));
  }

  /**
   * Factory method to create a deadline version where the path following is the deadline and intake
   * continues until the path is done.
   */
  public static Command deadlineVersion(
      Drive drive, Intake intake, SpinDexer spinDexer, String pathName) {

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return Commands.deadline(
        AutoBuilder.followPath(path), new IntakeUntilFullCommand(intake, spinDexer));
  }
}
