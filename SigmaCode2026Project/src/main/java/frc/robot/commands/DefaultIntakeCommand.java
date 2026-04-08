package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeMech;

public class DefaultIntakeCommand extends Command {
  private IntakeMech intakeMech;

  public DefaultIntakeCommand(IntakeMech intakeMech) {
    this.intakeMech = intakeMech;
    addRequirements(this.intakeMech);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeMech.stopRollerMotor();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
