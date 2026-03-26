package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.OscillateType;
import frc.robot.subsystems.intake.Intake.IntakeAngle;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeMotorVoltage = 0.0;
        public double intakeMotorVelocity = 0.0;
        public double intakeMotorCurrent = 0.0;
        public double intakeMotorTemp = 0.0;

        public double angleMotorPosition = 0.0;
        public double angleMotorVoltage = 0.0;
        public double angleMotorVelocity = 0.0;
        public double angleMotorCurrent = 0.0;
        public double angleMotorPIDSetpoint = 0.0;
        public double intakeEncoder = 0.0;
        public boolean isAngleMotorStalled = false;

        public boolean atHome = false;
        
    }

    public default void updateInputs(IntakeIOInputs inputs) {};

    public default void setIntakeMode(IntakeMode mode) {};

    public default void manualSetIntakeVoltage(double voltage) {};

    public default void manualSetIntakeAngle(double angle) {};

    public default void stop() {};

    public default void setOscillate(OscillateType type) {};
}
