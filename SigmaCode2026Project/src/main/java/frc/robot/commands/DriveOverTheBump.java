// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveOverTheBump extends Command {
  private Drive swerveDrive;

  private boolean gyroTilted;

  private double gyroFlatCooldown;
  /** Creates a new DriveOverTheBump. */
  public DriveOverTheBump(Drive swerveDrive) {
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyroTilted = false;
    gyroFlatCooldown = 20;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(swerveDrive.getRoll()) > 8) {
      gyroTilted = true;
    }
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(5.07, 0, 0);

    swerveDrive.runVelocityFieldRelative(fieldSpeeds);

    if (Math.abs(swerveDrive.getRoll()) < 3) {
      gyroFlatCooldown--;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (gyroTilted && Math.abs(swerveDrive.getRoll()) < 3 && gyroFlatCooldown < 0) {
      return true;
    } else {
      return false;
    }
  }
}
