package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {

  private final DCMotorSim driveMotor;
  private final DCMotorSim turnMotor;

  private boolean driveCloseLoop = false;
  private boolean turnCloseLoop = false;
  private final PIDController driveController = new PIDController(driveSimP, 0, 0);
  private final PIDController turnController = new PIDController(turnSimP, 0, 0);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {

    this.driveMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    this.turnMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveCloseLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveMotor.getAngularVelocityRadPerSec());

    } else {
      driveController.reset();
    }
    if (turnCloseLoop) {
      turnAppliedVolts = turnController.calculate(turnMotor.getAngularPositionRad());

    } else {
      turnController.reset();
    }
    driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12, 12));
    turnMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12, 12));
    driveMotor.update(0.02);
    turnMotor.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveMotor.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveMotor.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs((driveMotor.getCurrentDrawAmps()));

    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnMotor.getAngularPosition());
    inputs.turnVelocityRadPerSec = turnMotor.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs((turnMotor.getCurrentDrawAmps()));

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveCloseLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnCloseLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveCloseLoop = true;
    driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  public void setTurnPos(Rotation2d rotation) {
    turnCloseLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
