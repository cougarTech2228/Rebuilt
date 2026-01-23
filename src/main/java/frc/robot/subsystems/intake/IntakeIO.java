package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeMotorVoltage = 0.0;
        public double intakeMotorVelocity = 0.0;
        public double intakeMotorCurrent = 0.0;

        public double deployMotorPosition = 0.0;
        public double deployMotorVoltage = 0.0;
        public double deployMotorVelocity = 0.0;
        public double deployMotorCurrent = 0.0;

        public boolean atHome = false;
    }
    // TODO: write update of inputs
    public default void updateInputs(IntakeIOInputs inputs) {};

    // Methods wanted, will need more
    public default void setIntakeMode(IntakeMode mode) {};

    public default void setIntakePosition(IntakePosition position) {};
}
