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
        boolean isClimberReady;
        boolean isClimbComplete;

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

    public default void extend(Climber.ClimberLevel level) {};
    public default void retract() {};
    public default void climb(Climber.ClimberLevel level) {};
    public default void descend() {};
    public default boolean isExtended(Climber.ClimberLevel level) {return false;};
    public default void homeExtension() {};
    public default void homeClimber() {};
    public default void stopClimber() {};
    public default void stopExtension() {};
}
