package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public double hopperVelocity = 0.0;
        public double hopperVoltage = 0.0;
        public double hopperCurrent = 0.0;
        public boolean hopperOccupied = false;
    }
    public default void updateInputs(HopperIOInputs inputs) {}
    public default void turnOn() {}
    public default void turnOff() {}
    public boolean safeToRetract();
}
