package frc.robot.subsystems.SpinDexer;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpinDexerIOSim implements SpinDexerIO {
  // Creating simulated motors to mimic the real motors
  private DCMotorSim spinDexer;
  private DCMotorSim spinDexerKicker;

  // Creating the systems that the motors would be used for to add some physics to the simulation
  private LinearSystem<N2, N1, N2> spinDexerPhysics =
      LinearSystemId.createDCMotorSystem(
          DCMotor.getNEO(1), 1.02, 1); // Have to grab the MOI and Gear ratio later
  private LinearSystem<N2, N1, N2> spinDexerKickerPhysics =
      LinearSystemId.createDCMotorSystem(
          DCMotor.getNEO(1), 1.02, 1); // Have to grab the MOI and Gear ratio later

  // Tracks the voltage that the motors would be set to
  private double spinDexerMotorVoltage;
  private double spinDexerKickerMotorVoltage;

  public SpinDexerIOSim() {
    // Creating the motors with its defined data
    spinDexer =
        new DCMotorSim(
            spinDexerPhysics,
            DCMotor.getNEO(1).withReduction(1)); // Have to grab the Gear ratio later
    spinDexerKicker =
        new DCMotorSim(
            spinDexerKickerPhysics,
            DCMotor.getNEO(1).withReduction(1)); // Have to grab the Gear ratio later

    // Setting the voltage of the motors to 0 at boot up
    spinDexerMotorVoltage = 0.0;
    spinDexerKickerMotorVoltage = 0.0;
  }

  // Updates all inputs that are going to be log with any data that the simulation receives
  @Override
  public void updateInputs(SpinDexerIOInputs inputs) {
    inputs.spinDexerDisconnected = false;
    inputs.spinDexerKickerDisconnected = false;

    inputs.spinDexerMotorVoltage = spinDexerMotorVoltage;
    inputs.spinDexerKickerMotorVoltage = spinDexerKickerMotorVoltage;

    inputs.spinDexerMotorCurrent = spinDexer.getCurrentDrawAmps();
    inputs.spinDexerKickerMotorCurrent = spinDexerKicker.getCurrentDrawAmps();

    // Updates every 20 ms loop
    spinDexer.update(0.02);
    spinDexerKicker.update(0.02);
  }

  @Override
  public void setSpinDexerSpeed(double speed) {
    spinDexerMotorVoltage =
        12 * speed; // Converts the percentage of speed to the amount of volts the motor would
    // receive to accomplish speed
    spinDexer.setInputVoltage(
        spinDexerMotorVoltage); // Sets the voltage we want the motor to run at
  }

  @Override
  public void setKickerSpeed(double speed) {
    spinDexerKickerMotorVoltage =
        12 * speed; // Converts the percentage of speed to the amount of volts the motor would
    // receive to accomplish speed
    spinDexerKicker.setInputVoltage(
        spinDexerKickerMotorVoltage); // Sets the voltage we want the motor to run at
  }

  @Override
  public void setSpinDexerVoltage(double volt) {
    spinDexerMotorVoltage = volt;
    spinDexer.setInputVoltage(spinDexerMotorVoltage);
  }

  @Override
  public void setKickerVoltage(double volt) {
    spinDexerKickerMotorVoltage = volt;
    spinDexerKicker.setInputVoltage(volt);
  }
}
