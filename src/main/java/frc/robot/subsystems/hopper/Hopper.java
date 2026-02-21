package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
	private final HopperIO io;

    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

	private boolean onIndexer = false; 
	private boolean onKicker = false;

	public enum HopperMode {
		ON,
		OFF
	}
	
	public Hopper(HopperIO io) {
		this.io = io;

		SmartDashboard.putBoolean("IndexerTest", false);
	}

    @Override
    public void periodic() {
        super.periodic();
    	io.updateInputs(hopperInputs);
        Logger.processInputs("Hopper", hopperInputs);
    }

	public boolean isOnHopper() {
		return this.onIndexer;
	}

	public boolean isOnKicker() {
		return this.onKicker;
	}

	public void indexerOn(boolean test) {
		this.onIndexer = true;
		this.io.indexerOn(test);
	}
	
	public void indexerOff() {
		this.onIndexer = false;
		this.io.indexerOff();
	}

	public void kickerOn(boolean test) {
		this.onKicker = true;
		this.io.kickerOn(test);
	}

	public void kickerOff() {
		this.onKicker = false;
		this.io.kickerOff();
	}

}


