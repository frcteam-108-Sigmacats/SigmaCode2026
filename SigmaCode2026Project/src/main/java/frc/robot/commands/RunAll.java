// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake.IntakeMech;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;
import frc.robot.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAll extends ParallelCommandGroup {
  /** Creates a new RunAll. */
  public RunAll(
      Shooter shooterMech, IntakeMech intakeMech, SpinDexerMech spinDexerMech, Drive swerveDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunIntakeCommand(intakeMech, swerveDrive),
        new DefaultShooter(shooterMech, swerveDrive, true),
        new TransferFuelToShooter(spinDexerMech, swerveDrive),
        new InstantCommand(() -> swerveDrive.setDriveState("Shoot")));
  }
}
