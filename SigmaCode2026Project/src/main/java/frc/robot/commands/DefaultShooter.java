// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultShooter extends Command {
  private Shooter shooterMech;
  private Drive swerveDrive;
  /** Creates a new DefaultShooter. */
  public DefaultShooter(Shooter shooterMech, Drive swerveDrive) {
    this.shooterMech = shooterMech;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooterMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d aimPoint =
        shooterMech.getAimPoint(shooterMech.getTargetPose(swerveDrive), swerveDrive);
    Translation2d diff = aimPoint.minus(swerveDrive.getPose().getTranslation());
    Rotation2d desiredAngle = Rotation2d.fromRadians(Math.atan2(diff.getY(), diff.getX()));
    desiredAngle =
        desiredAngle.minus(swerveDrive.getPose().getRotation().minus(Rotation2d.k180deg));
    if (desiredAngle.getDegrees() > 105) {
      desiredAngle = new Rotation2d(desiredAngle.getRadians() - (2 * Math.PI));
      System.out.println("Greater than 91");
    } else if (desiredAngle.getDegrees() < -260) {
      desiredAngle = new Rotation2d(desiredAngle.getRadians() - (2 * Math.PI));
    }
    Logger.recordOutput("DESIRED TURRET ANGLE", desiredAngle.getDegrees());
    Logger.recordOutput("ROBOT DISTANCE FROM HUB", diff.getNorm());
    Logger.recordOutput("Aim Point", new Pose2d(aimPoint, new Rotation2d()));
    shooterMech.setTurretAngle(desiredAngle);
    shooterMech.setShooterSpeed(diff.getNorm());
    shooterMech.setHoodAngle(diff.getNorm());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
