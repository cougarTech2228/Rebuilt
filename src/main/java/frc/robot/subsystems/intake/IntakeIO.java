package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakeAngle;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeMotorVoltage = 0.0;
        public double intakeMotorVelocity = 0.0;
        public double intakeMotorCurrent = 0.0;

        public double angleMotorPosition = 0.0;
        public double angleMotorVoltage = 0.0;
        public double angleMotorVelocity = 0.0;
        public double angleMotorCurrent = 0.0;

        public boolean atHome = false;
        
    }

    public default void updateInputs(IntakeIOInputs inputs) {};

    public default void setIntakeMode(IntakeMode mode) {};

    public default void setIntakeAngle(IntakeAngle angle) {};

    public default void manualSetIntakeVelocity(double velocity) {};

    public default void manualSetIntakeAngle(double angle) {};
}
