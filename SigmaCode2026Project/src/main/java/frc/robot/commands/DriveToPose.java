// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.TrajectoryMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private Drive swerveDrive;
  private int poseCount;
  private List<TrajectoryMap> poses;
  private HolonomicDriveController swerveDriveController =
      new HolonomicDriveController(
          DriveConstants.xyLinearPIDController,
          DriveConstants.xyLinearPIDController,
          DriveConstants.thetaPIDController);

  private boolean start;

  private boolean isFinished;
  /**
   * Creates a new DefaultIntakeCommand.
   *
   * @param swerveDrive the subsystem that will be used
   * @param poses the list of poses for this path
   * @param start Indicates if this pose/path is the start for our Auto
   */
  public DriveToPose(Drive swerveDrive, List<TrajectoryMap> poses, boolean start) {
    this.swerveDrive = swerveDrive;
    this.poses = poses;
    this.start = start;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      for (int i = 0; i < poses.size(); i++) {
        poses.get(i).setPose(FlippingUtil.flipFieldPose(poses.get(i).getPose()));
      }
    }
    if (start) {
      poseCount = 1;
      swerveDrive.resetOdometry(poses.get(0).getPose());
    } else {
      poseCount = 0;
    }

    swerveDriveController.setTolerance(new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (poseCount == poses.size()) {
      isFinished = true;
      return;
    }
    ChassisSpeeds speeds =
        swerveDriveController.calculate(
            swerveDrive.getPose(),
            poses.get(poseCount).getPose(),
            poses.get(poseCount).getMaxSpeed(),
            poses.get(poseCount).getHeading());
    swerveDrive.runVelocity(speeds);
    Logger.recordOutput("/Auto/CurrentTargetPose", poses.get(poseCount).getPose());

    if (swerveDriveController.atReference()) {
      poseCount++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
