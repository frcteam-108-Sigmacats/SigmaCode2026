package frc.robot.KxNSwerve;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KxNSwerve.Constants.SwerveDriveConstants;

public class SwerveDrive extends SubsystemBase {
  
  
  public final Swervemodule fLeftModule = new Swervemodule(SwerveDriveConstants.fLDriveMotorID,SwerveDriveConstants.fLTurnMotorID, SwerveDriveConstants.fLAbsEncoderOffset);

  public final Swervemodule fRightModule = new Swervemodule(SwerveDriveConstants.fRDriveMotorID,SwerveDriveConstants.fRTurnMotorID, SwerveDriveConstants.fRAbsEncoderOffset);

  public final Swervemodule bLeftModule = new Swervemodule(SwerveDriveConstants.bLDriveMotorID,SwerveDriveConstants.bLTurnMotorID, SwerveDriveConstants.bLAbsEncoderOffset);

  public final Swervemodule bRightModule = new Swervemodule(SwerveDriveConstants.bRDriveMotorID,SwerveDriveConstants.bRTurnMotorID, SwerveDriveConstants.bRAbsEncoderOffset);

  private Pigeon2 gyro = new Pigeon2(1, "*");

  private static SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private Swervemodule [] modules = {fLeftModule, fRightModule, bLeftModule, bRightModule};

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    gyro.getConfigurator().apply(new Pigeon2Configuration());
 
    gyro.clearStickyFault_BootDuringEnable();

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.swerveKinematics, getHeading(), getModulePosition(), new Pose2d());

  }
    
    public Rotation2d getHeading(){
      return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
    }

    public Pose2d getPose(){
      return swerveDrivePoseEstimator.getEstimatedPosition();
    }
    public void zeroHeading(Rotation2d heading){
      swerveDrivePoseEstimator.resetPosition(getHeading(), getModulePosition(), new Pose2d(getPose().getTranslation(), heading));
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
      SwerveModuleState[] swerveModuleStates =
          SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation, 
                                  getYaw().minus(Rotation2d.kPi)
                              )
                              : new ChassisSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation)
                              );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.kMaxSpeedMPS);
        fLeftModule.setDesiredState(swerveModuleStates[0]);
        fRightModule.setDesiredState(swerveModuleStates[1]);
        bLeftModule.setDesiredState(swerveModuleStates[2]);
        bRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.kMaxSpeedMPS);
        fLeftModule.setDesiredState(desiredStates[0]);
        fRightModule.setDesiredState(desiredStates[1]);
        bLeftModule.setDesiredState(desiredStates[2]);
        bRightModule.setDesiredState(desiredStates[3]);
    }

     public SwerveModuleState[] getModuleStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for(int i = 0; i < states.length; i++){
        states[i] = modules[i].getState();
      }
      return states;
      
    }

    //Gets the modules position (How much did the robot drive forward or backwards and left or right in meters and the direction the wheels are facing)
    public SwerveModulePosition[] getModulePosition(){
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for(int i = 0; i < positions.length; i++){
        positions[i] = modules[i].getModulePosition();
      }
      return positions;
    }

    //Resetting the Pose Estimator
      public void resetEstimator(Pose2d pose){
        swerveDrivePoseEstimator.resetPosition(getHeading(), getModulePosition(), pose);
      }

    //Sets the drive encoders to 0
    public void resetEncoders(){
      fLeftModule.resetEncoders();
      fRightModule.resetEncoders();
      bLeftModule.resetEncoders();
      bRightModule.resetEncoders();
    }

  //Same thing as getting the heading but is not restricted to 360 degrees
  public Rotation2d getYaw(){
    double yaw = gyro.getYaw().getValueAsDouble();
    return (SwerveDriveConstants.gyroReversed) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
  }

  public ChassisSpeeds getSpeeds(){
    SwerveModuleState[] states = getModuleStates();
    return SwerveDriveConstants.swerveKinematics.toChassisSpeeds(states);
  }
}

