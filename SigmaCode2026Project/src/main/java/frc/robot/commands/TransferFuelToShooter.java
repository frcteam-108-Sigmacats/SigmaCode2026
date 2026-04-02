// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants.ShooterStatus;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransferFuelToShooter extends Command {
  private SpinDexerMech spinDexerMech;
  private Drive swerveDrive;
  private int counter;
  /** Creates a new TransferFuelToShooter. */
  public TransferFuelToShooter(SpinDexerMech spinDexerMech, Drive swerveDrive) {
    this.spinDexerMech = spinDexerMech;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.spinDexerMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.setDriveState(ShooterStatus.SHOOT);
    spinDexerMech.setShooterStatus(swerveDrive.getDriveState());
    counter = 40;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter--;
    if (counter < 40) {
      spinDexerMech.setSpinDexerClockwise();
      spinDexerMech.setKickerForward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (swerveDrive.getDriveState() == ShooterStatus.SHOOT) {
      swerveDrive.setDriveState(ShooterStatus.DRIVE);
      spinDexerMech.setShooterStatus(swerveDrive.getDriveState());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
