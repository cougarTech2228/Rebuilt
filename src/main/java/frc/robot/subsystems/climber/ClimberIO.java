package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        boolean isExtensionHome;
        boolean isClimberHome;
        boolean hasExtensionHomed;
        boolean hasClimberHomed;
        boolean isClimberExtended;

        double climberMotorPosition;
        double climberMotorCurrent;
        double climberMotorTemp;
        double climberMotorPIDTarget;

        double extendMotorPosition;
        double extendMotorCurrent;
        double extendMotorTemp;
        double extendMotorPIDTarget;
    }
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void extend() {};
    public default void retract() {};
    public default void climb() {};
    public default void descend() {};
    public default boolean isExtended() {return false;};
    public default void homeExtension() {};
    public default void homeClimber() {};
}
