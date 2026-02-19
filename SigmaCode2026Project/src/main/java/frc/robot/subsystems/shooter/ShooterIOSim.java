package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Simulation implementation for the shooter subsystem. */
public class ShooterIOSim implements ShooterIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final FlywheelSim leftFlywheelSim;
  private final FlywheelSim rightFlywheelSim;
  private final DCMotorSim turretSim;
  private final DCMotorSim hoodSim;

  private double leftTargetRPM = 0.0;
  private double rightTargetRPM = 0.0;
  private double turretTargetDeg = 0.0;
  private double hoodTargetDeg = 0.0;

  public ShooterIOSim() {
    leftFlywheelSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.02);
    rightFlywheelSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.02);
    turretSim = new DCMotorSim(DCMotor.getNeoVortex(1), 100.0, 0.01);
    hoodSim = new DCMotorSim(DCMotor.getNeo550(1), 50.0, 0.004);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Simplified velocity control for flywheels
    double leftError = leftTargetRPM - leftFlywheelSim.getAngularVelocityRPM();
    double leftVoltage = leftError * 0.01;
    leftVoltage = Math.max(-12.0, Math.min(12.0, leftVoltage));
    leftFlywheelSim.setInputVoltage(leftVoltage);
    leftFlywheelSim.update(LOOP_PERIOD_SECS);

    double rightError = rightTargetRPM - rightFlywheelSim.getAngularVelocityRPM();
    double rightVoltage = rightError * 0.01;
    rightVoltage = Math.max(-12.0, Math.min(12.0, rightVoltage));
    rightFlywheelSim.setInputVoltage(rightVoltage);
    rightFlywheelSim.update(LOOP_PERIOD_SECS);

    // Simplified position control for turret and hood
    double turretError = turretTargetDeg - Math.toDegrees(turretSim.getAngularPositionRad());
    double turretVoltage = turretError * 0.1;
    turretVoltage = Math.max(-12.0, Math.min(12.0, turretVoltage));
    turretSim.setInputVoltage(turretVoltage);
    turretSim.update(LOOP_PERIOD_SECS);

    double hoodError = hoodTargetDeg - Math.toDegrees(hoodSim.getAngularPositionRad());
    double hoodVoltage = hoodError * 0.1;
    hoodVoltage = Math.max(-12.0, Math.min(12.0, hoodVoltage));
    hoodSim.setInputVoltage(hoodVoltage);
    hoodSim.update(LOOP_PERIOD_SECS);

    // Update inputs
    double leftRPM = leftFlywheelSim.getAngularVelocityRPM();
    double rightRPM = rightFlywheelSim.getAngularVelocityRPM();

    inputs.leftShooterVelocityRPM = leftRPM;
    inputs.rightShooterVelocityRPM = rightRPM;

    inputs.leftShooterAppliedVolts = leftVoltage;
    inputs.rightShooterAppliedVolts = rightVoltage;

    inputs.leftShooterCurrentAmps = leftFlywheelSim.getCurrentDrawAmps();
    inputs.rightShooterCurrentAmps = rightFlywheelSim.getCurrentDrawAmps();

    inputs.leftShooterTempCelsius = 25.0;
    inputs.rightShooterTempCelsius = 25.0;

    inputs.turretPositionDeg = Math.toDegrees(turretSim.getAngularPositionRad());
    inputs.turretVelocityDegPerSec = Math.toDegrees(turretSim.getAngularVelocityRadPerSec());
    inputs.turretAppliedVolts = turretVoltage;
    inputs.turretCurrentAmps = turretSim.getCurrentDrawAmps();
    inputs.turretTempCelsius = 25.0;

    inputs.hoodPositionDeg = Math.toDegrees(hoodSim.getAngularPositionRad());
    inputs.hoodVelocityDegPerSec = Math.toDegrees(hoodSim.getAngularVelocityRadPerSec());
    inputs.hoodAppliedVolts = hoodVoltage;
    inputs.hoodCurrentAmps = hoodSim.getCurrentDrawAmps();
    inputs.hoodTempCelsius = 25.0;
  }

  @Override
  public void setFlywheelVelocity(double leftRPM, double rightRPM) {
    leftTargetRPM = leftRPM;
    rightTargetRPM = rightRPM;
  }

  @Override
  public void setTurretPosition(double degrees) {
    turretTargetDeg = degrees;
  }

  @Override
  public void setHoodPosition(double degrees) {
    hoodTargetDeg = degrees;
  }

  @Override
  public void stop() {
    leftTargetRPM = 0.0;
    rightTargetRPM = 0.0;
    leftFlywheelSim.setInputVoltage(0.0);
    rightFlywheelSim.setInputVoltage(0.0);
    turretSim.setInputVoltage(0.0);
    hoodSim.setInputVoltage(0.0);
  }
}
