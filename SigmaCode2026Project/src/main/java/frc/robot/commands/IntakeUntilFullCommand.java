package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.SpinDexer;

/**
 * Continuously intakes game pieces until all SpinDexer slots are full. Automatically indexes to the
 * next empty slot after each intake.
 */
public class IntakeUntilFullCommand extends Command {
  private final Intake intake;
  private final SpinDexer spinDexer;

  private int previousOccupiedCount = 0;
  private boolean waitingForIndex = false;

  /**
   * Creates a new IntakeUntilFullCommand.
   *
   * @param intake The intake subsystem
   * @param spinDexer The spin dexer subsystem
   */
  public IntakeUntilFullCommand(Intake intake, SpinDexer spinDexer) {
    this.intake = intake;
    this.spinDexer = spinDexer;

    addRequirements(intake, spinDexer);
  }

  @Override
  public void initialize() {
    previousOccupiedCount = spinDexer.getOccupiedSlotCount();
    waitingForIndex = false;
  }

  @Override
  public void execute() {
    int currentOccupiedCount = spinDexer.getOccupiedSlotCount();

    if (currentOccupiedCount >= 3) {
      // All slots full, stop
      intake.stop();
      return;
    }

    if (!waitingForIndex) {
      // Run intake
      intake.runVoltage(12.0 * 0.8);

      // Check if we got a new game piece
      if (intake.hasGamePiece() && currentOccupiedCount > previousOccupiedCount) {
        // New game piece detected, start indexing
        waitingForIndex = true;
        intake.runVoltage(1.0); // Hold
        spinDexer.rotateToNextSlot();
        previousOccupiedCount = currentOccupiedCount;
      }
    } else {
      // Wait for indexing to complete
      // Simple timer approach - wait 1 second
      intake.runVoltage(1.0); // Hold during index

      // After a brief period, resume intaking
      waitingForIndex = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    // Finish when all slots are full
    return spinDexer.getOccupiedSlotCount() >= 3;
  }
}
