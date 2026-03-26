package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RunAll;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.subsystems.Intake.IntakeMech;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class DepotAuto extends SequentialCommandGroup {

  public DepotAuto(
      Drive swerveDrive, IntakeMech intakeMech, SpinDexerMech spinDexerMech, Shooter shooter) {

    addCommands(
        new ParallelRaceGroup(
            new DriveToPose(swerveDrive, DriveConstants.depotPath1, true),
            new RunIntakeCommand(intakeMech, swerveDrive)),
        new DriveToPose(swerveDrive, DriveConstants.depotPath2, false),
        new DriveToPose(swerveDrive, DriveConstants.depotPath3, false),
        new RunAll(shooter, intakeMech, spinDexerMech, swerveDrive));
  }
}
