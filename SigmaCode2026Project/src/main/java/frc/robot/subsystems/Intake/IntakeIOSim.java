package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeRotatorMotor;
  private DCMotorSim intakeRollerMotor;

  private LinearSystem<N2, N1, N2> intakeRotatorMotorPhysics =
      LinearSystemId.createSingleJointedArmSystem(
          DCMotor.getNEO(1), 1.02, 3); // Have to grab the MOI and Gear Ratio later
  private LinearSystem<N2, N1, N2> intakeRollerMotorPhysics =
      LinearSystemId.createDCMotorSystem(
          DCMotor.getNeoVortex(1), 1.02, 1); // Have to grab the MOI and Gear Ratio later

  private PIDController intakePID;

  private double intakeRotatorMotorVoltage;
  private double intakeRollerMotorVoltage;

  public IntakeIOSim() {
    intakeRotatorMotor =
        new DCMotorSim(intakeRotatorMotorPhysics, DCMotor.getNEO(1).withReduction(1.02));
    intakeRollerMotor =
        new DCMotorSim(intakeRollerMotorPhysics, DCMotor.getNeoVortex(1).withReduction(1.02));

    intakePID = new PIDController(0.3, 0.0, 0.0);

    intakeRotatorMotorVoltage = 0.0;
    intakeRollerMotorVoltage = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeActuaterDisconnected = false;
    inputs.intakeRollerDisconnected = false;

    inputs.intakeActuaterVoltage = intakeRotatorMotorVoltage;
    inputs.intakeRollerVoltage = intakeRollerMotorVoltage;

    inputs.intakeActuaterAngle = new Rotation2d(intakeRotatorMotor.getAngularPositionRad());

    inputs.intakeActuaterCurrent = intakeRotatorMotor.getCurrentDrawAmps();
    inputs.intakeRollerCurrent = intakeRollerMotor.getCurrentDrawAmps();

    // Updates every 20 ms loop
    intakeRotatorMotor.update(0.02);
    intakeRollerMotor.update(0.02);
  }

  @Override
  public void setIntakeActuaterSpeed(double speed) {
    intakeRotatorMotorVoltage = 12 * speed;
    intakeRotatorMotor.setInputVoltage(intakeRotatorMotorVoltage);
  }

  @Override
  public void setIntakeActuaterPosition(double position) {
    Rotation2d currentAngle = new Rotation2d(intakeRotatorMotor.getAngularPositionRad());
    intakeRotatorMotorVoltage = intakePID.calculate(currentAngle.getDegrees(), position);
    intakeRotatorMotor.setInputVoltage(intakeRotatorMotorVoltage);
    intakeRotatorMotor.setAngularVelocity(intakeRotatorMotorVoltage * (2 * Math.PI / 60));
  }

  @Override
  public void setIntakeRollerSpeed(double speed) {
    intakeRollerMotorVoltage = 12 * speed;
    intakeRollerMotor.setInputVoltage(intakeRollerMotorVoltage);
  }
}
