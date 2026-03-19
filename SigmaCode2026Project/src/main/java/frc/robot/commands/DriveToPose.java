// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.TrajectoryMap;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
    private Drive swerveDrive;
    private int poseCount;
    private List<TrajectoryMap> poses;
    private HolonomicDriveController swerveDriveController = new HolonomicDriveController(DriveConstants.xyLinearPIDController, 
        DriveConstants.xyLinearPIDController, DriveConstants.thetaPIDController);
    
    private boolean isFinished;
  /** Creates a new DefaultIntakeCommand. */
  public DriveToPose(Drive swerveDrive, List<TrajectoryMap> poses) {
    this.swerveDrive = swerveDrive;
    this.poses = poses;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    poseCount = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(poseCount < poses.size()){
        ChassisSpeeds speeds = swerveDriveController.calculate(swerveDrive.getPose(), poses.get(poseCount).getPose(), poses.get(poseCount).getMaxSpeed(), poses.get(poseCount).getHeading());
        swerveDrive.runVelocity(speeds);

        if(swerveDriveController.atReference()){
            poseCount++;
        }
    }
    isFinished = true;

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
