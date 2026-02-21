// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//intake mech
package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMech extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private Alert actuationDisconnected = new Alert("Actuater Motor Is Disconnected", AlertType.kError) ;
  private Alert rollerDisconnected = new Alert("Roller Motor Is Disconnected", AlertType.kError);
  /** Creates a new ExampleSubsystem. */
  public IntakeMech(IntakeIO io) {
    this.io = io;
  }

  public void setIntakeOut(){
    io.setIntakeActuaterPosition(IntakeConstants.intakeOutPosition);
  }
  public void setIntakeIn(){
    io.setIntakeActuaterPosition(IntakeConstants.intakeInPosition);
  }
  public void setIntakeRoller(){
    io.setIntakeRollerSpeed(IntakeConstants.rollerSpeedPercentage);
  }
  public void stopActuaterMotor(){
    io.setActuaterVoltage(0);
  }
  public void stopRollerMotor(){
    io.setRollerVoltage(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
