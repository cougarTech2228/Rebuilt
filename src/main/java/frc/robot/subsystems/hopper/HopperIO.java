package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public double indexerVoltage = 0.0;
        public double indexerVelocity = 0.0;
        public double indexerAmps = 0.0;

        public double kickVoltage = 0.0;
        public double kickVelocity = 0.0;
        public double kickAmps = 0.0;

        public double extensionVoltage = 0.0;
        public double extensionVelocity = 0.0;
        public double extensionCurrent = 0.0;

        public boolean canHome = false;
    }
    public default void updateInputs(HopperIOInputs inputs) {}
    public default void indexerOn(boolean test) {}
    public default void indexerOff() {}
    
    public default boolean safeToRetract() {
            return false;};
}


