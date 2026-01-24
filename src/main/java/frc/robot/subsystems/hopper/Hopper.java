package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
	private final HopperIO io;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

	private boolean on = false;

	public Hopper(HopperIO io) {
		this.io = io;
	}

    @Override
    public void periodic() {
        super.periodic();
    	io.updateInputs(hopperInputs);
        Logger.processInputs("Hopper", hopperInputs);
    }

	public boolean isOn() {
		return this.on;
	}

	public void turnOn() {
		this.on = true;
		this.io.turnOn();
	}
	
	public void turnOff() {
		this.on = false;
		this.io.turnOff();
	}

	public boolean safeToRetract() {
		return this.io.safeToRetract();
	}
}
