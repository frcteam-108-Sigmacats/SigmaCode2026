package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics-simulation implementation of {@link TurretIO}.
 *
 * <p>Three independent {@link DCMotorSim} instances model:
 *
 * <ul>
 *   <li>Turret rotation (Vortex)
 *   <li>Left shooter wheel (Kraken X60)
 *   <li>Right shooter wheel (Kraken X60)
 *   <li>Hood angle (Neo 550)
 * </ul>
 */
public class TurretIOSim implements TurretIO {

  private static final double DT = 0.02; // 50 Hz loop period (seconds)

  // ── Simulated plants ──────────────────────────────────────────────────────
  private final DCMotorSim turretSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(turretMotorModel, turretMOI, turretGearRatio),
          turretMotorModel);

  private final DCMotorSim shooterLeftSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              shooterMotorModel, shooterWheelMOI, shooterWheelGearRatio),
          shooterMotorModel);

  private final DCMotorSim shooterRightSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              shooterMotorModel, shooterWheelMOI, shooterWheelGearRatio),
          shooterMotorModel);

  private final DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(hoodMotorModel, hoodMOI, hoodGearRatio),
          hoodMotorModel);

  // ── Applied voltages ──────────────────────────────────────────────────────
  private double turretVolts = 0.0;
  private double shooterLeftVolts = 0.0;
  private double shooterRightVolts = 0.0;
  private double hoodVolts = 0.0;

  // ── Closed-loop controllers ───────────────────────────────────────────────
  private final PIDController turretPID = new PIDController(turretSimKp, 0.0, turretSimKd);
  private final PIDController shooterPID = new PIDController(shooterSimKp, 0.0, shooterSimKd);
  private final PIDController hoodPID = new PIDController(hoodSimKp, 0.0, hoodSimKd);

  private boolean turretClosedLoop = false;
  private boolean shooterClosedLoop = false;
  private boolean hoodClosedLoop = false;

  private double turretSetpointRad = 0.0;
  private double shooterSetpointRadPerSec = 0.0;
  private double hoodSetpointDeg = 0.0;

  public TurretIOSim() {
    // Turret wraps; hood does not
    turretPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ── updateInputs ─────────────────────────────────────────────────────────

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // Run closed-loop calculations before stepping the plant
    if (turretClosedLoop) {
      turretVolts = turretPID.calculate(turretSim.getAngularPositionRad(), turretSetpointRad);
    }
    if (shooterClosedLoop) {
      double effort =
          shooterPID.calculate(
              shooterLeftSim.getAngularVelocityRadPerSec(), shooterSetpointRadPerSec);
      shooterLeftVolts = effort;
      shooterRightVolts = effort;
    }
    if (hoodClosedLoop) {
      hoodVolts =
          hoodPID.calculate(Math.toDegrees(hoodSim.getAngularPositionRad()), hoodSetpointDeg);
    }

    // Apply voltages and step all plants
    turretSim.setInputVoltage(MathUtil.clamp(turretVolts, -12.0, 12.0));
    shooterLeftSim.setInputVoltage(MathUtil.clamp(shooterLeftVolts, -12.0, 12.0));
    shooterRightSim.setInputVoltage(MathUtil.clamp(shooterRightVolts, -12.0, 12.0));
    hoodSim.setInputVoltage(MathUtil.clamp(hoodVolts, -12.0, 12.0));

    turretSim.update(DT);
    shooterLeftSim.update(DT);
    shooterRightSim.update(DT);
    hoodSim.update(DT);

    // Turret
    inputs.turretConnected = true;
    inputs.turretPositionRad = turretSim.getAngularPositionRad();
    inputs.turretVelocityRadPerSec = turretSim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVolts = turretVolts;
    inputs.turretCurrentAmps = Math.abs(turretSim.getCurrentDrawAmps());

    // Shooter wheels
    inputs.shooterLeftConnected = true;
    inputs.shooterRightConnected = true;
    inputs.shooterLeftVelocityRadPerSec = shooterLeftSim.getAngularVelocityRadPerSec();
    inputs.shooterRightVelocityRadPerSec = shooterRightSim.getAngularVelocityRadPerSec();
    inputs.shooterLeftAppliedVolts = shooterLeftVolts;
    inputs.shooterRightAppliedVolts = shooterRightVolts;
    inputs.shooterLeftCurrentAmps = Math.abs(shooterLeftSim.getCurrentDrawAmps());
    inputs.shooterRightCurrentAmps = Math.abs(shooterRightSim.getCurrentDrawAmps());

    // Hood – hoodSim stores radians internally; the IO contract exposes degrees
    inputs.hoodConnected = true;
    inputs.hoodPositionDeg = Math.toDegrees(hoodSim.getAngularPositionRad());
    inputs.hoodVelocityDegPerSec = Math.toDegrees(hoodSim.getAngularVelocityRadPerSec());
    inputs.hoodAppliedVolts = hoodVolts;
    inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
  }

  // ── Turret rotation ───────────────────────────────────────────────────────

  @Override
  public void setTurretOpenLoop(double outputVolts) {
    turretClosedLoop = false;
    turretPID.reset();
    turretVolts = outputVolts;
  }

  @Override
  public void setTurretPosition(double angleRad) {
    turretClosedLoop = true;
    turretSetpointRad = angleRad;
  }

  // ── Shooter wheels ────────────────────────────────────────────────────────

  @Override
  public void setShooterOpenLoop(double outputVolts) {
    shooterClosedLoop = false;
    shooterPID.reset();
    shooterLeftVolts = outputVolts;
    shooterRightVolts = outputVolts;
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    shooterClosedLoop = true;
    shooterSetpointRadPerSec = velocityRadPerSec;
  }

  // ── Hood ──────────────────────────────────────────────────────────────────

  @Override
  public void setHoodOpenLoop(double outputVolts) {
    hoodClosedLoop = false;
    hoodPID.reset();
    hoodVolts = outputVolts;
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    hoodClosedLoop = true;
    hoodSetpointDeg = MathUtil.clamp(angleDeg, hoodMinDeg, hoodMaxDeg);
  }
}
