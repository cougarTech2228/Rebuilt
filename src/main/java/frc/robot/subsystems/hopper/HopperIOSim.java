package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class HopperIOSim implements HopperIO {
    private boolean isTurnedOn = false;
    // private final DigitalInput sensor = new DigitalInput(Constants.hopperOccupationDIO);

    public HopperIOSim() {
        
    }

    public void updateInputs(HopperIOInputs inputs) {
        // inputs.hopperVoltage = isTurnedOn ? HopperConstants.SpinningVoltage : 0.0;
        // inputs.hopperOccupied = sensor.get();
    }

    public void turnOn() {
        isTurnedOn = true;
    }

    public void turnOff() {
        isTurnedOn = false;
    }

    public boolean safeToRetract() {
        return true;
    }
}
