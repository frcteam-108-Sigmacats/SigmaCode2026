package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics-simulation implementation (IOSim) for the Turret subsystem.
 *
 * <p>All three mechanism groups are modelled with WPILib {@link DCMotorSim}:
 * <ul>
 *   <li>Turret rotation (Vortex sim)
 *   <li>Shooter left wheel (Kraken X60 sim)
 *   <li>Shooter right wheel (Kraken X60 sim)
 *   <li>Hood (Neo 550 sim)
 * </ul>
 */
public class TurretIOSim implements TurretIO {

  private static final double DT = 0.02; // 50 Hz loop period

  // ── Sim motors ───────────────────────────────────────────────────────────
  private final DCMotorSim turretSim;
  private final DCMotorSim shooterLeftSim;
  private final DCMotorSim shooterRightSim;
  private final DCMotorSim hoodSim;

  // ── Applied voltages ──────────────────────────────────────────────────────
  private double turretAppliedVolts = 0.0;
  private double shooterLeftAppliedVolts = 0.0;
  private double shooterRightAppliedVolts = 0.0;
  private double hoodAppliedVolts = 0.0;

  // ── Closed-loop flags & controllers ──────────────────────────────────────
  private boolean turretClosedLoop = false;
  private boolean shooterClosedLoop = false;
  private boolean hoodClosedLoop = false;

  private final PIDController turretPID =
      new PIDController(turretSimKp, 0.0, turretSimKd);
  private final PIDController shooterPID =
      new PIDController(shooterSimKp, 0.0, shooterSimKd);
  private final PIDController hoodPID =
      new PIDController(hoodSimKp, 0.0, hoodSimKd);

  private double turretSetpointRad = 0.0;
  private double shooterSetpointRadPerSec = 0.0;
  private double hoodSetpointDeg = 0.0;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turretMotorModel, turretMOI, turretGearRatio),
            turretMotorModel);

    shooterLeftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(shooterMotorModel, shooterWheelMOI, shooterWheelGearRatio),
            shooterMotorModel);

    shooterRightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(shooterMotorModel, shooterWheelMOI, shooterWheelGearRatio),
            shooterMotorModel);

    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(hoodMotorModel, hoodMOI, hoodGearRatio),
            hoodMotorModel);

    // Turret can wrap around; hood cannot
    turretPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    // ── Turret closed-loop ────────────────────────────────────────────────
    if (turretClosedLoop) {
      turretAppliedVolts =
          turretPID.calculate(turretSim.getAngularPositionRad(), turretSetpointRad);
    }

    // ── Shooter closed-loop ───────────────────────────────────────────────
    if (shooterClosedLoop) {
      double effort = shooterPID.calculate(
          shooterLeftSim.getAngularVelocityRadPerSec(), shooterSetpointRadPerSec);
      shooterLeftAppliedVolts = effort;
      shooterRightAppliedVolts = effort;
    }

    // ── Hood closed-loop ──────────────────────────────────────────────────
    if (hoodClosedLoop) {
      hoodAppliedVolts =
          hoodPID.calculate(
              Math.toDegrees(hoodSim.getAngularPositionRad()), hoodSetpointDeg);
    }

    // ── Step simulations ──────────────────────────────────────────────────
    turretSim.setInputVoltage(MathUtil.clamp(turretAppliedVolts, -12.0, 12.0));
    shooterLeftSim.setInputVoltage(MathUtil.clamp(shooterLeftAppliedVolts, -12.0, 12.0));
    shooterRightSim.setInputVoltage(MathUtil.clamp(shooterRightAppliedVolts, -12.0, 12.0));
    hoodSim.setInputVoltage(MathUtil.clamp(hoodAppliedVolts, -12.0, 12.0));

    turretSim.update(DT);
    shooterLeftSim.update(DT);
    shooterRightSim.update(DT);
    hoodSim.update(DT);

    // ── Populate inputs ───────────────────────────────────────────────────
    inputs.turretConnected = true;
    inputs.turretPositionRad = turretSim.getAngularPositionRad();
    inputs.turretVelocityRadPerSec = turretSim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVolts = turretAppliedVolts;
    inputs.turretCurrentAmps = Math.abs(turretSim.getCurrentDrawAmps());

    inputs.shooterLeftConnected = true;
    inputs.shooterRightConnected = true;
    inputs.shooterLeftVelocityRadPerSec = shooterLeftSim.getAngularVelocityRadPerSec();
    inputs.shooterRightVelocityRadPerSec = shooterRightSim.getAngularVelocityRadPerSec();
    inputs.shooterLeftAppliedVolts = shooterLeftAppliedVolts;
    inputs.shooterRightAppliedVolts = shooterRightAppliedVolts;
    inputs.shooterLeftCurrentAmps = Math.abs(shooterLeftSim.getCurrentDrawAmps());
    inputs.shooterRightCurrentAmps = Math.abs(shooterRightSim.getCurrentDrawAmps());

    inputs.hoodConnected = true;
    // hoodSim angular position is stored in radians internally; convert to degrees for the API
    inputs.hoodPositionDeg = Math.toDegrees(hoodSim.getAngularPositionRad());
    inputs.hoodVelocityDegPerSec = Math.toDegrees(hoodSim.getAngularVelocityRadPerSec());
    inputs.hoodAppliedVolts = hoodAppliedVolts;
    inputs.hoodCurrentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
  }

  // ── Turret Commands ───────────────────────────────────────────────────────

  @Override
  public void setTurretOpenLoop(double outputVolts) {
    turretClosedLoop = false;
    turretPID.reset();
    turretAppliedVolts = outputVolts;
  }

  @Override
  public void setTurretPosition(double angleRad) {
    turretClosedLoop = true;
    turretSetpointRad = angleRad;
  }

  // ── Shooter Wheel Commands ────────────────────────────────────────────────

  @Override
  public void setShooterOpenLoop(double outputVolts) {
    shooterClosedLoop = false;
    shooterPID.reset();
    shooterLeftAppliedVolts = outputVolts;
    shooterRightAppliedVolts = outputVolts;
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    shooterClosedLoop = true;
    shooterSetpointRadPerSec = velocityRadPerSec;
  }

  // ── Hood Commands ─────────────────────────────────────────────────────────

  @Override
  public void setHoodOpenLoop(double outputVolts) {
    hoodClosedLoop = false;
    hoodPID.reset();
    hoodAppliedVolts = outputVolts;
  }

  @Override
  public void setHoodPosition(double angleDeg) {
    hoodClosedLoop = true;
    hoodSetpointDeg = MathUtil.clamp(angleDeg, hoodMinDeg, hoodMaxDeg);
  }
}
