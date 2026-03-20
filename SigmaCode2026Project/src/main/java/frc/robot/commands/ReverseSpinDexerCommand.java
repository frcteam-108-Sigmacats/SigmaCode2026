// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinDexer.SpinDexerMech;

public class ReverseSpinDexerCommand extends Command {

  private SpinDexerMech spinDexerMech;

  public ReverseSpinDexerCommand(SpinDexerMech spinDexerMech) {
    this.spinDexerMech = spinDexerMech;
    addRequirements(this.spinDexerMech);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Opposite of TransferFuelToShooter:
    // Shooter feed uses CounterClockwise + KickerForward,
    // so reverse uses Clockwise + KickerReverse
    spinDexerMech.setSpinDexerClockwise();
    spinDexerMech.setSpinDexerCounterClockwise();
  }

  @Override
  public void end(boolean interrupted) {
    spinDexerMech.stopSpinDexer();
    spinDexerMech.stopKickers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
