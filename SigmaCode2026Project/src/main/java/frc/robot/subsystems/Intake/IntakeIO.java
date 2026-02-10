package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public boolean intakeRotatorDisconnected = false;
        public double intakeRotatorCurrent = 0.0;
        public double intakeRotatorVoltage = 0.0;
        public Rotation2d intakeRotatorAngle = new Rotation2d();

        public boolean intakeRollerDisconnected = false;
        public double intakeRollerCurrent = 0.0;
        public double intakeRollerVoltage = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs){}
    public default void setIntakeRotatorSpeed(double speed){}
    public default void setIntakeRotatorPosition(double position){}
    public default void setIntakeRollerSpeed(double speed){}
}
