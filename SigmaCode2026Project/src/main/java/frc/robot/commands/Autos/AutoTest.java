package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Intake.IntakeMech;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class AutoTest extends SequentialCommandGroup{

    public AutoTest(Drive swerveDrive, IntakeMech intakeMech, SpinDexerMech spinDexerMech, Shooter shooter){
        addCommands(
            new DriveToPose(swerveDrive, DriveConstants.testPath, true)
        );
    }

}
