package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Custom class to store a pose with its heading and max speed */
public class TrajectoryMap {
  private Pose2d targetPose;
  private Rotation2d heading;
  private double maxSpeed;
  /**
   * Instantiates the targetPose and heading and maxSpeed given
   *
   * @param targetPose Stores the pose the robot should be going to
   * @param heading The heading of where the pose should be pointing
   * @param maxSpeed The speed in mps
   */
  public TrajectoryMap(Pose2d targetPose, Rotation2d heading, double maxSpeed) {
    this.targetPose = targetPose;
    this.heading = heading;
    this.maxSpeed = maxSpeed;
  }
  /**
   * @return the pose the robot needs to drive to
   */
  public Pose2d getPose() {
    return targetPose;
  }
  /**
   * Sets the pose in case the poses needs to be flipped for Red Alliance
   *
   * @param pose the new pose to be set to
   */
  public void setPose(Pose2d pose) {
    this.targetPose = pose;
  }
  /**
   * @return the heading the pose should be facing
   */
  public Rotation2d getHeading() {
    return heading;
  }
  /**
   * @return the max speed the robot should be going at this point
   */
  public double getMaxSpeed() {
    return maxSpeed;
  }
}
