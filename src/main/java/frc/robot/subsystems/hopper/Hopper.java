package frc.robot.subsystems.hopper;

public class Hopper {
	private final HopperIO io;

	private boolean on = false;

	public Hopper(HopperIO io) {
		this.io = io;
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
}
