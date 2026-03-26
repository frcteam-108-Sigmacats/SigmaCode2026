package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RunAll;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.subsystems.Intake.IntakeMech;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class StationAuto extends SequentialCommandGroup{
    public StationAuto(Drive swerveDrive, Shooter shooterMech, IntakeMech intakeMech, SpinDexerMech spinDexerMech){
        addCommands(
        new ParallelRaceGroup(new DriveToPose(swerveDrive, DriveConstants.stationPath1, true), 
            new RunIntakeCommand(intakeMech, swerveDrive), new DefaultShooter(shooterMech, swerveDrive, false)), 
        new DriveToPose(swerveDrive, DriveConstants.stationPath2, false), 
        new DriveToPose(swerveDrive, DriveConstants.stationPath3, false),
        new ParallelCommandGroup(new DriveToPose(swerveDrive, DriveConstants.stationPath4, false), 
            new RunAll(shooterMech, intakeMech, spinDexerMech, swerveDrive)));
    }

}
