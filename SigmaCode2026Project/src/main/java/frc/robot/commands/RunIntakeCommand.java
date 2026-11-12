package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeMech;
import frc.robot.subsystems.drive.Drive;

public class RunIntakeCommand extends Command {
  private IntakeMech intakeMech;
  private Drive swerveDrive;

  public RunIntakeCommand(IntakeMech intakeMech, Drive swerveDrive) {
    this.intakeMech = intakeMech;
    this.swerveDrive = swerveDrive;
    addRequirements(this.intakeMech);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeMech.setIntakeRoller();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
