// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.KxNSwerve;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.fasterxml.jackson.databind.Module.SetupContext;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KxNSwerve.Constants.SwerveDriveConstants;



public class Swervemodule{
  private TalonFX drivemotor;
  private SparkMax turnmotor;

  private AbsoluteEncoder absEncoder;

  private TalonFXConfiguration drirveMotorconfig = new TalonFXConfiguration();

  private VelocityVoltage velocity = new VelocityVoltage(0);

  private SparkClosedLoopController turnPIDcontroller;
  private SparkMaxConfig turnmotorConfig = new SparkMaxConfig();

  private double absAngleoffset;

  private SwerveModuleState m_desiredstate = new SwerveModuleState();

  /** Creates a new ExampleSubsystem. */
  public Swervemodule(int drivemotorID, int turnmotorID, double angleOffset ) {
    drivemotor = new TalonFX(drivemotorID, "*");
    turnmotor = new SparkMax(turnmotorID,MotorType.kBrushless);

    drivemotor.getConfigurator().apply(new TalonFXConfiguration());

    drirveMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turnmotorConfig.idleMode(IdleMode.kCoast);

    drirveMotorconfig.CurrentLimits.StatorCurrentLimitEnable = true;
    drirveMotorconfig.CurrentLimits.StatorCurrentLimit = SwerveDriveConstants.driveMotorCurrentLimit; 
  
    drirveMotorconfig.Feedback.SensorToMechanismRatio = SwerveDriveConstants.kDrivingMotorReduction;

    drirveMotorconfig.Slot0.kP = SwerveDriveConstants.drivemotorP;
    drirveMotorconfig.Slot0.kI = SwerveDriveConstants.drivemotorI;
    drirveMotorconfig.Slot0.kD = SwerveDriveConstants.drivemotorD;

    drirveMotorconfig.Slot0.kV = 1 / SwerveDriveConstants.krakenRPM; // helps tells the pid controller with grav into equ//
   
    turnmotorConfig.idleMode(IdleMode.kCoast);
    turnmotorConfig.smartCurrentLimit(SwerveDriveConstants.turnmotorCurrentLimit);

    turnmotorConfig.absoluteEncoder.positionConversionFactor(SwerveDriveConstants.turningFactor);
    turnmotorConfig.absoluteEncoder.velocityConversionFactor(SwerveDriveConstants.turningFactor / 60.0);
    turnmotorConfig.absoluteEncoder.inverted(true);

    turnmotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    turnmotorConfig.closedLoop.pid(SwerveDriveConstants.turnmotorP,SwerveDriveConstants.turnmotorI,SwerveDriveConstants.turnmotorD);

    turnmotorConfig.closedLoop.outputRange(-1.0, 1.0);
    turnmotorConfig.closedLoop.positionWrappingEnabled(true);
    turnmotorConfig.closedLoop.positionWrappingInputRange(0, SwerveDriveConstants.turningFactor);

    absEncoder = turnmotor.getAbsoluteEncoder();

    turnPIDcontroller  = turnmotor.getClosedLoopController();

    drivemotor.getConfigurator().apply(drirveMotorconfig);
    turnmotor.configure(turnmotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    absAngleoffset = angleOffset;
    m_desiredstate.angle = new Rotation2d(absEncoder.getPosition());
    drivemotor.setPosition(0);
    
  }


  public SwerveModuleState getState(){
    return new SwerveModuleState(Units.rotationsToRadians(drivemotor.getVelocity().getValueAsDouble()) * Units.inchesToMeters(3.0) * 0.5,
    new Rotation2d(absEncoder.getPosition() - absAngleoffset));
  }
  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(Units.rotationsToRadians(drivemotor.getPosition().getValueAsDouble()) * Units.inchesToMeters(3.0) * 0.5,
   new Rotation2d(absEncoder.getPosition()- absAngleoffset));
  }
   public void setDesiredState(SwerveModuleState desiredstate){
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredstate.speedMetersPerSecond;
    correctedDesiredState.angle = desiredstate.angle.plus(Rotation2d.fromRadians(absAngleoffset));

    correctedDesiredState.optimize(new Rotation2d(absEncoder.getPosition()));

    drivemotor.setControl(velocity.withVelocity(Units.radiansToRotations(correctedDesiredState.speedMetersPerSecond / (Units.inchesToMeters(3.0) * 0.5))));

    turnPIDcontroller.setSetpoint(correctedDesiredState.angle.getRadians(),
    ControlType.kPosition);
    
   }

   public double getMotorVoltage(){
    return drivemotor.getMotorVoltage().getValueAsDouble();
   }

public void resetEncoders() {
    drivemotor.setPosition(0);
}

}
