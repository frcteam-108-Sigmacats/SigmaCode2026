package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;

/**
 * Simulation implementation for the swerve drive subsystem. Uses WPILib's DCMotorSim for physics
 * simulation.
 */
public class DriveIOSim implements DriveIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  // Simulated drive motors
  private final DCMotorSim[] driveMotorSims = new DCMotorSim[4];
  private final DCMotorSim[] steerMotorSims = new DCMotorSim[4];

  // Simulated gyro
  private double gyroYawDeg = 0.0;
  private double gyroYawVelocityDegPerSec = 0.0;

  // Applied voltages
  private double[] driveAppliedVolts = new double[4];
  private Rotation2d[] steerSetpoints = new Rotation2d[4];

  public DriveIOSim() {
    // Initialize drive motor sims (Kraken X60)
    for (int i = 0; i < 4; i++) {
      driveMotorSims[i] =
          new DCMotorSim(DCMotor.getKrakenX60Foc(1), DriveConstants.kDriveGearRatio, 0.025); // MOI

      steerMotorSims[i] =
          new DCMotorSim(DCMotor.getNeo550(1), DriveConstants.kSteerGearRatio, 0.004); // MOI

      steerSetpoints[i] = new Rotation2d();
    }
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Update drive motors
    for (int i = 0; i < 4; i++) {
      driveMotorSims[i].update(LOOP_PERIOD_SECS);
      steerMotorSims[i].update(LOOP_PERIOD_SECS);

      // Drive
      double driveRotations = driveMotorSims[i].getAngularPositionRad() / (2 * Math.PI);
      inputs.moduleDrivePositionsMeters[i] =
          (driveRotations / DriveConstants.kDriveGearRatio) * DriveConstants.kWheelCircumference;

      double driveRPM = driveMotorSims[i].getAngularVelocityRPM();
      inputs.moduleDriveVelocitiesMetersPerSec[i] =
          (driveRPM / 60.0 / DriveConstants.kDriveGearRatio) * DriveConstants.kWheelCircumference;

      inputs.moduleDriveCurrentAmps[i] = Math.abs(driveMotorSims[i].getCurrentDrawAmps());
      inputs.moduleDriveTempCelsius[i] = 25.0;

      // Steer
      double steerRotations = steerMotorSims[i].getAngularPositionRotations();
      inputs.moduleSteerPositionsDeg[i] = steerRotations * 360.0;
      inputs.moduleSteerVelocitiesDegPerSec[i] =
          steerMotorSims[i].getAngularVelocityRPM() * 6.0; // RPM to deg/sec
      inputs.moduleSteerCurrentAmps[i] = Math.abs(steerMotorSims[i].getCurrentDrawAmps());
      inputs.moduleSteerTempCelsius[i] = 25.0;
    }

    // Update gyro (simplified - just sum the module velocities)
    double averageVelocity = 0;
    for (int i = 0; i < 4; i++) {
      averageVelocity += inputs.moduleDriveVelocitiesMetersPerSec[i];
    }
    averageVelocity /= 4.0;

    gyroYawVelocityDegPerSec = averageVelocity * 10.0; // Simplified angular velocity
    gyroYawDeg += gyroYawVelocityDegPerSec * LOOP_PERIOD_SECS;
    gyroYawDeg = MathUtil.inputModulus(gyroYawDeg, 0, 360);

    inputs.gyroYawDegrees = gyroYawDeg;
    inputs.gyroPitchDegrees = 0.0;
    inputs.gyroRollDegrees = 0.0;
    inputs.gyroYawVelocityDegPerSec = gyroYawVelocityDegPerSec;
    inputs.gyroConnected = true;

    // Set odometry data (simplified for sim)
    inputs.odometryTimestamps = new double[] {inputs.gyroYawDegrees};
    inputs.odometryYawPositions = new double[] {inputs.gyroYawDegrees};
    inputs.odometryDrivePositionsMeters =
        new double[][] {inputs.moduleDrivePositionsMeters.clone()};
    inputs.odometrySteerPositions =
        new Rotation2d[] {
          Rotation2d.fromDegrees(inputs.moduleSteerPositionsDeg[0]),
          Rotation2d.fromDegrees(inputs.moduleSteerPositionsDeg[1]),
          Rotation2d.fromDegrees(inputs.moduleSteerPositionsDeg[2]),
          Rotation2d.fromDegrees(inputs.moduleSteerPositionsDeg[3])
        };
  }

  @Override
  public void setDriveVoltage(double[] voltages) {
    for (int i = 0; i < 4; i++) {
      driveAppliedVolts[i] = MathUtil.clamp(voltages[i], -12.0, 12.0);
      driveMotorSims[i].setInputVoltage(driveAppliedVolts[i]);
    }
  }

  @Override
  public void setDriveVelocity(double[] velocitiesMetersPerSec) {
    // Simplified - just use feedforward voltage
    for (int i = 0; i < 4; i++) {
      double voltage = velocitiesMetersPerSec[i] / DriveConstants.kMaxSpeedMetersPerSecond * 12.0;
      setDriveVoltage(new double[] {voltage, voltage, voltage, voltage});
    }
  }

  @Override
  public void setSteerPosition(Rotation2d[] positions) {
    for (int i = 0; i < 4; i++) {
      steerSetpoints[i] = positions[i];

      // Simple P controller for steering
      double currentAngle = steerMotorSims[i].getAngularPositionRotations();
      double error = positions[i].getRotations() - currentAngle;
      double voltage = error * 12.0; // Proportional control
      voltage = MathUtil.clamp(voltage, -12.0, 12.0);
      steerMotorSims[i].setInputVoltage(voltage);
    }
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    // Not applicable in simulation
  }

  @Override
  public void setSteerBrakeMode(boolean enable) {
    // Not applicable in simulation
  }

  @Override
  public void stop() {
    for (int i = 0; i < 4; i++) {
      driveMotorSims[i].setInputVoltage(0.0);
      steerMotorSims[i].setInputVoltage(0.0);
    }
  }
}
