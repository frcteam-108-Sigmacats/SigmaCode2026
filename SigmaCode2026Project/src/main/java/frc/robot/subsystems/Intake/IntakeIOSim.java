package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeRollerMotor;
  private double intakeRollerMotorVoltage = 0.0;

  public IntakeIOSim() {
    intakeRollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.02, 1),
            DCMotor.getKrakenX60(1).withReduction(1.02));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeRollerDisconnected = false;
    inputs.intakeRollerVoltage = intakeRollerMotorVoltage;
    inputs.intakeRollerCurrent = intakeRollerMotor.getCurrentDrawAmps();
    intakeRollerMotor.update(0.02);
  }

  @Override
  public void setIntakeRollerSpeed(double speed) {
    intakeRollerMotorVoltage = 12 * speed;
    intakeRollerMotor.setInputVoltage(intakeRollerMotorVoltage);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    intakeRollerMotorVoltage = voltage;
    intakeRollerMotor.setInputVoltage(intakeRollerMotorVoltage);
  }
}
