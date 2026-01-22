package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakeMode;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;

        public boolean atHome = false;
        public boolean deployed = false;
    }
    // TODO: write update of inputs
    public default void updateInputs(IntakeIOInputs inputs) {};

    // Methods wanted
    public default void setIntakeMode(IntakeMode mode) {};
    // public default void setIntakePosition()
}
